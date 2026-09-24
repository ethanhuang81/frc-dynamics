"""Single rigid arm calculator; Python standard library only. See README.md."""
import argparse
import csv
import html
import json
import math
from pathlib import Path

LB = 0.45359237
INCH = 0.0254
GRAVITY = 9.80665


class Model:
    def __init__(self, cfg):
        self.c = cfg
        self.validate()
        m = cfg['motor']
        self.mass = cfg['mass_lb'] * LB
        self.radius = cfg['com_radius_in'] * INCH
        self.j_com = cfg['com_inertia_lb_in2'] * LB * INCH**2
        self.j_arm = self.j_com + self.mass*self.radius**2
        self.r = m['nominal_voltage']/m['stall_current_a']
        self.kt = m['stall_torque_nm']/m['stall_current_a']
        self.free_speed = m['free_rpm']*2*math.pi/60
        self.ke = (m['nominal_voltage']-self.r*m['free_current_a'])/self.free_speed
        # Viscous motor loss matches no-load current at datasheet free speed.
        self.motor_b = self.kt*m['free_current_a']/self.free_speed
        self.n = cfg['motor_count']
        self.rb = cfg['battery_resistance_ohm']+cfg['wiring_resistance_ohm']
        self.v0 = cfg['battery_v']-self.rb*cfg['other_supply_current_a']
        self.start = math.radians(cfg['start_deg'])
        self.target = math.radians(cfg['target_deg'])
        self.offset = math.radians(cfg['com_offset_deg'])

    def validate(self):
        c = self.c
        def finite_tree(x):
            if isinstance(x, dict):
                for v in x.values(): finite_tree(v)
            elif isinstance(x, list):
                for v in x: finite_tree(v)
            elif isinstance(x, (float, int)) and not math.isfinite(x):
                raise ValueError('All numeric inputs must be finite')
        finite_tree(c)
        for k in ('mass_lb','stator_limit_a','supply_limit_a_per_motor','battery_v',
                  'duration_min_s','duration_max_s','sample_dt_s','position_tolerance_deg',
                  'velocity_tolerance_deg_s','settle_dwell_s','controller_bandwidth_rad_s'):
            if c[k] <= 0: raise ValueError(k+' must be positive')
        for k in ('com_radius_in','com_inertia_lb_in2','battery_resistance_ohm',
                  'wiring_resistance_ohm','other_supply_current_a',
                  'output_viscous_friction_nm_per_rad_s','extra_output_inertia_kg_m2',
                  'regen_limit_a_per_motor'):
            if c[k] < 0: raise ValueError(k+' must be nonnegative')
        if c['com_inertia_lb_in2']+c['mass_lb']*c['com_radius_in']**2 <= 0:
            raise ValueError('Pivot inertia must be positive')
        if type(c['motor_count']) is not int or c['motor_count'] < 1:
            raise ValueError('motor_count must be a positive integer')
        if not c['ratios'] or any(g <= 0 for g in c['ratios']):
            raise ValueError('Ratios must be positive')
        if not 0 < c['electrical_headroom_fraction'] <= 1:
            raise ValueError('Headroom must be in (0,1]')
        if c['duration_scan_factor'] <= 1 or c['duration_max_s'] < c['duration_min_s']:
            raise ValueError('Invalid duration search')
        if c['hold_s'] < c['settle_dwell_s']:
            raise ValueError('Hold must cover settling dwell')
        if c['sample_dt_s'] > min(0.001, c['duration_min_s']/20):
            raise ValueError('sample_dt_s too large for this model')
        m = c['motor']
        for k in ('nominal_voltage','free_rpm','stall_current_a','stall_torque_nm'):
            if m[k] <= 0: raise ValueError('Invalid motor '+k)
        if not 0 <= m['free_current_a'] < m['stall_current_a'] or m['rotor_inertia_kg_m2'] < 0:
            raise ValueError('Invalid motor loss or inertia')
        if c['battery_v'] <= (c['battery_resistance_ohm']+c['wiring_resistance_ohm'])*c['other_supply_current_a']:
            raise ValueError('Background load collapses bus')

    def inertia(self, ratio):
        return (self.j_arm+self.c['extra_output_inertia_kg_m2']
                +self.n*self.c['motor']['rotor_inertia_kg_m2']*ratio**2)

    def gravity_load(self, q):
        return self.mass*GRAVITY*self.radius*math.cos(q+self.offset)

    def damping(self, ratio):
        return self.c['output_viscous_friction_nm_per_rad_s']+self.n*self.motor_b*ratio**2

    def electrical(self, current, speed):
        """Requested current -> required terminal V and consistent battery V.

        Vbus^2 - (Voc-Rb*Iother)*Vbus + Rb*Pmotor = 0.
        Select high-voltage branch. Regeneration is ideal and reversible.
        """
        terminal = self.r*current+self.ke*speed
        power = self.n*terminal*current
        disc = self.v0**2-4*self.rb*power
        if disc < 0: return None
        bus = (self.v0+math.sqrt(disc))/2
        supply = terminal*current/bus
        return dict(bus_v=bus, terminal_v=terminal, stator_a=current,
                    supply_a=supply, battery_a=self.c['other_supply_current_a']+self.n*supply,
                    motor_rpm=speed*60/(2*math.pi), duty=terminal/bus)

    def violations(self, p, margin=1.0):
        if p is None: return ['battery_power']
        bad = []
        if abs(p['duty']) > margin+1e-10: bad.append('voltage')
        if abs(p['stator_a']) > self.c['stator_limit_a']*margin+1e-10: bad.append('stator_current')
        if p['supply_a'] > self.c['supply_limit_a_per_motor']*margin+1e-10: bad.append('supply_current')
        if -p['supply_a'] > self.c['regen_limit_a_per_motor']*margin+1e-10: bad.append('regeneration')
        return bad

    def requested_current(self, q, v, acceleration, ratio):
        torque = self.inertia(ratio)*acceleration+self.gravity_load(q)+self.damping(ratio)*v
        return torque/(self.n*ratio*self.kt)

    def limited_current(self, request, speed):
        p = self.electrical(request, speed)
        if not self.violations(p): return request, p
        # The zero-current anchor must itself be voltage-feasible. Reject overspeed
        # rather than claiming an impossible open-circuit current clamp.
        zero = self.electrical(0, speed)
        if self.violations(zero):
            raise ValueError('Back-EMF exceeds bus: overspeed behavior unsupported')
        lo, hi = 0.0, 1.0
        # Search the connected feasible region from zero; sampling detects a
        # regen-limit gap before the plugging branch at large reverse current.
        for k in range(1, 65):
            f = k/64
            if self.violations(self.electrical(request*f, speed)):
                lo, hi = (k-1)/64, f
                break
        for _ in range(35):
            mid = (lo+hi)/2
            if self.violations(self.electrical(request*mid, speed)): hi = mid
            else: lo = mid
        actual = request*lo
        return actual, self.electrical(actual, speed)


def profile(t, duration, start, target):
    if t >= duration: return target, 0.0, 0.0
    s = max(0, t/duration)
    d = target-start
    return (start+d*(10*s**3-15*s**4+6*s**5),
            d/duration*(30*s**2-60*s**3+30*s**4),
            d/duration**2*(60*s-180*s**2+120*s**3))


def feasible_profile(model, ratio, duration, samples=600):
    for k in range(samples+1):
        q, v, a = profile(duration*k/samples, duration, model.start, model.target)
        i = model.requested_current(q, v, a, ratio)
        if model.violations(model.electrical(i, ratio*v), model.c['electrical_headroom_fraction']):
            return False
    return True


def choose_duration(model, ratio):
    """First feasible duration on a geometric grid; no global-optimum claim."""
    duration = model.c['duration_min_s']
    maximum = model.c['duration_max_s']
    while True:
        if feasible_profile(model, ratio, duration): return duration
        if duration >= maximum: return None
        duration = min(maximum, duration*model.c['duration_scan_factor'])


def simulate(model, ratio, duration, dt=None):
    """Sampled computed-torque PD; current held per sample, RK4 mechanics.

    Ideal instantaneous current regulation, not Talon firmware emulation.
    Electrical feasibility is re-evaluated at each mechanical RK4 stage.
    """
    controller_dt = model.c['sample_dt_s']
    substeps = max(1, math.ceil(controller_dt/(dt or controller_dt)))
    dt = controller_dt/substeps
    q, v = model.start, 0.0
    rows = []
    draw = regen = 0.0
    controls = math.ceil((duration+model.c['hold_s'])/controller_dt)
    steps = controls*substeps
    end = steps*dt
    wn = model.c['controller_bandwidth_rad_s']
    j, b = model.inertia(ratio), model.damping(ratio)
    def rhs(qi, vi, request):
        current, p = model.limited_current(request, ratio*vi)
        torque = model.n*ratio*model.kt*current
        acceleration = (torque-model.gravity_load(qi)-b*vi)/j
        return vi, acceleration, p, torque
    for k in range(steps+1):
        t = k*dt
        qr, vr, ar = profile(t, duration, model.start, model.target)
        if k % substeps == 0:
            command_a = ar+2*wn*(vr-v)+wn**2*(qr-q)
            request = model.requested_current(q, v, command_a, ratio)
        k1 = rhs(q, v, request)
        rows.append(dict(time_s=t, angle_deg=math.degrees(q), reference_deg=math.degrees(qr),
                         velocity_deg_s=math.degrees(v), acceleration_rad_s2=k1[1],
                         output_em_torque_nm=k1[3], gravity_load_nm=model.gravity_load(q),
                         **k1[2]))
        if k == steps: break
        k2 = rhs(q+dt*k1[0]/2, v+dt*k1[1]/2, request)
        k3 = rhs(q+dt*k2[0]/2, v+dt*k2[1]/2, request)
        k4 = rhs(q+dt*k3[0], v+dt*k3[1], request)
        # Integrate power within each held-command interval, not across its
        # discontinuous boundary to the next command.
        for stage, weight in ((k1,1),(k2,2),(k3,2),(k4,1)):
            power = model.n*stage[2]['bus_v']*stage[2]['supply_a']
            draw += max(power,0)*dt*weight/6
            regen += max(-power,0)*dt*weight/6
        q += dt*(k1[0]+2*k2[0]+2*k3[0]+k4[0])/6
        v += dt*(k1[1]+2*k2[1]+2*k3[1]+k4[1])/6
    # Require an uninterrupted terminal interval after profile completion.
    final_run = None
    for row in rows:
        good = (row['time_s'] >= duration
                and abs(row['angle_deg']-math.degrees(model.target)) <= model.c['position_tolerance_deg']
                and abs(row['velocity_deg_s']) <= model.c['velocity_tolerance_deg_s'])
        if not good: final_run = None
        elif final_run is None: final_run = row['time_s']
    completed = final_run is not None and end-final_run >= model.c['settle_dwell_s']
    summary = dict(ratio=ratio, profile_s=duration, status='completed' if completed else 'not_settled',
                   completion_s=final_run+model.c['settle_dwell_s'] if completed else None,
                   peak_stator_a=max(abs(x['stator_a']) for x in rows),
                   peak_supply_a=max(x['supply_a'] for x in rows),
                   peak_regen_a=max(0,-min(x['supply_a'] for x in rows)),
                   min_bus_v=min(x['bus_v'] for x in rows),
                   max_bus_v=max(x['bus_v'] for x in rows),
                   peak_motor_rpm=max(abs(x['motor_rpm']) for x in rows),
                   peak_output_em_torque_nm=max(abs(x['output_em_torque_nm']) for x in rows),
                   peak_acceleration_rad_s2=max(abs(x['acceleration_rad_s2']) for x in rows),
                   draw_j=draw, regen_j=regen, net_j=draw-regen,
                   final_error_deg=rows[-1]['angle_deg']-math.degrees(model.target))
    return rows, summary


def save_csv(path, rows):
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def chart(rows, keys, title, unit):
    colors = ['#38bdf8','#fb923c','#a78bfa']
    values = [row[k] for row in rows for k in keys]
    lo, hi = min(0,min(values)), max(values)
    if hi-lo < 1e-9: hi = lo+1
    width, height = 720, 225
    x0, y0, pw, ph = 65, 25, 630, 155
    xmax = max(row['time_s'] for row in rows) or 1
    parts = [f'<h3>{html.escape(title)}</h3><svg viewBox="0 0 {width} {height}" role="img" aria-label="{html.escape(title)}">']
    for f in (0,0.5,1):
        y = y0+ph*(1-f)
        parts.append(f'<path d="M{x0} {y}h{pw}" stroke="#334155"/><text x="4" y="{y+4}">{lo+f*(hi-lo):.3g}</text>')
    stride = max(1,len(rows)//1400)
    sampled = rows[::stride]
    if sampled[-1] is not rows[-1]: sampled = sampled+[rows[-1]]
    for key, color in zip(keys,colors):
        points = ' '.join(f'{x0+r["time_s"]/xmax*pw:.2f},{y0+ph*(hi-r[key])/(hi-lo):.2f}' for r in sampled)
        parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.6"/>')
    parts.append(f'<text x="65" y="205">0 s</text><text x="635" y="205">{xmax:.3f} s</text></svg>')
    parts.append('<p>'+html.escape(unit)+' · '+' / '.join(f'<span style="color:{color}">{html.escape(key)}</span>' for key,color in zip(keys,colors))+'</p>')
    return ''.join(parts)


def report(model, summaries, best, trace, convergence, path):
    headings = ['Ratio','Profile (s)','Completion (s)','Stator peak (A)','Supply peak (A)','Bus min (V)','Status']
    table = '<table><tr>'+''.join('<th>'+x+'</th>' for x in headings)+'</tr>'
    for s in summaries:
        vals = [s['ratio'],s.get('profile_s'),s.get('completion_s'),s.get('peak_stator_a'),s.get('peak_supply_a'),s.get('min_bus_v'),s['status']]
        table += '<tr>'+''.join('<td>'+ (f'{x:.4g}' if isinstance(x,(int,float)) else html.escape(str(x or '—')))+'</td>' for x in vals)+'</tr>'
    table += '</table>'
    plots = ''
    if best:
        for keys,title,unit in [(['angle_deg','reference_deg'],'Angle and reference','degrees'),
                                (['velocity_deg_s'],'Output velocity','degrees/s'),
                                (['output_em_torque_nm','gravity_load_nm'],'Torque at output','N m; electromagnetic torque before modeled losses'),
                                (['stator_a','supply_a'],'Motor and supply current','A per motor; negative supply = regeneration'),
                                (['bus_v','terminal_v'],'Battery and motor voltage','V')]:
            plots += chart(trace,keys,title,unit)
    content = f'''<!doctype html><meta charset="utf-8"><title>FRC arm calculator</title>
<style>body{{background:#0f172a;color:#e2e8f0;font:16px system-ui;margin:40px auto;max-width:1000px;padding:0 20px}}h1,h2{{color:#f8fafc}}p,li{{line-height:1.6}}table{{width:100%;border-collapse:collapse}}td,th{{text-align:left;padding:10px;border-bottom:1px solid #334155}}svg{{width:100%;background:#172033;border-radius:10px}}svg text{{fill:#94a3b8;font:12px system-ui}}a{{color:#38bdf8}}.notice{{background:#312e1e;padding:18px;border-radius:10px}}</style>
<h1>Rigid-arm dynamics calculator</h1><p>0° is horizontal; positive rotation lifts the COM. Local Python prototype.</p>
<div class="notice"><b>Provisional predictions, not a hardware-validated gear recommendation.</b><ul>
{''.join('<li>'+html.escape(x)+'</li>' for x in model.c['notes'])}</ul></div>
<h2>Your arm</h2><p>Mass: {model.mass:.8f} kg · COM radius: {model.radius:.6f} m · COM inertia: {model.j_com:.9f} kg m² · Pivot inertia: {model.j_arm:.9f} kg m².<br>Horizontal gravity torque: {model.gravity_load(model.start):.6f} N m.</p>
<h2>Reduction sweep</h2><p>Each ratio uses the first electrically feasible quintic profile on a {model.c['duration_scan_factor']:.3f}× duration grid, with {100*(1-model.c['electrical_headroom_fraction']):.1f}% electrical headroom. Forward simulation checks tracking and terminal settling. Completion includes {model.c['settle_dwell_s']:.3f} s dwell. This is a profile-family comparison, not global time optimization.</p>
{table}<h2>Selected trace</h2><p>{html.escape('Best completed sampled candidate: '+str(best['ratio'])+':1' if best else 'No completed candidate found.')}</p>{plots}
<h2>Numerical check</h2><pre>{html.escape(json.dumps(convergence,indent=2))}</pre>
<p>CSV traces and summary.json are saved alongside this report. All input parameters are in arm.json; rerun Python after editing.</p>
<h2>Sources</h2><ul>{''.join('<li><a href="'+html.escape(url,quote=True)+'">'+html.escape(url)+'</a></li>' for url in model.c['sources'])}</ul>'''
    path.write_text(content)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=Path(__file__).with_name('arm.json'))
    parser.add_argument('--output',type=Path,default=Path(__file__).with_name('results'))
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    model = Model(cfg)
    args.output.mkdir(parents=True,exist_ok=True)
    summaries, traces = [], {}
    for ratio in cfg['ratios']:
        duration = choose_duration(model,ratio)
        if duration is None:
            summaries.append(dict(ratio=ratio,status='no_feasible_profile'))
            continue
        try:
            trace, summary = simulate(model,ratio,duration)
            save_csv(args.output/f'trace_{ratio:g}.csv',trace)
            traces[ratio] = trace
            summaries.append(summary)
            print(f'{ratio:g}:1  profile={duration:.4f}s  {summary["status"]}',flush=True)
        except ValueError as exc:
            summaries.append(dict(ratio=ratio,status=str(exc)))
    good = [s for s in summaries if s['status']=='completed']
    best = min(good,key=lambda s:s['completion_s']) if good else None
    convergence = {}
    if best:
        fine_trace, fine = simulate(model,best['ratio'],best['profile_s'],cfg['sample_dt_s']/2)
        convergence = {k:abs(fine[k]-best[k]) for k in ('peak_stator_a','peak_supply_a','min_bus_v','net_j')}
        convergence['fine_status'] = fine['status']
        convergence['completion_difference_s'] = abs(fine['completion_s']-best['completion_s']) if fine['completion_s'] is not None else None
        convergence['profile_6000_samples_feasible'] = feasible_profile(model,best['ratio'],best['profile_s'],6000)
        convergence['passed'] = bool(fine['status']=='completed'
            and convergence['completion_difference_s'] <= cfg['sample_dt_s']*1.01
            and convergence['peak_stator_a'] <= max(0.1,0.01*best['peak_stator_a'])
            and convergence['peak_supply_a'] <= max(0.1,0.01*best['peak_supply_a'])
            and convergence['min_bus_v'] <= 0.01
            and convergence['net_j'] <= max(0.001,0.005*abs(best['net_j']))
            and convergence['profile_6000_samples_feasible'])
        save_csv(args.output/'best_trace_half_dt.csv',fine_trace)
    output = dict(inputs=cfg,mass_kg=model.mass,pivot_inertia_kg_m2=model.j_arm,
                  candidates=summaries,best=best,convergence=convergence)
    (args.output/'summary.json').write_text(json.dumps(output,indent=2,allow_nan=False))
    report(model,summaries,best,traces[best['ratio']] if best else [],convergence,args.output/'report.html')
    print(json.dumps(dict(best=best,convergence=convergence),indent=2))


if __name__=='__main__': main()
