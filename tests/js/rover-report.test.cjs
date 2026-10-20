const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const script = fs.readFileSync(path.join(__dirname, '../../src/roboforge/hardware/assets/rover-report.js'), 'utf8');
class Element {
  constructor(tag) { this.tag = tag; this.children = []; this.attributes = {}; this.listeners = {}; this.value = ''; this.style = {setProperty() {}}; }
  append(...children) { this.children.push(...children); }
  replaceChildren(...children) { this.children = children; }
  setAttribute(key, value) { this.attributes[key] = String(value); }
  addEventListener(name, callback) { this.listeners[name] = callback; }
  getBoundingClientRect() { return {left: 0, width: 1000}; }
  fire(name, event = {}) { this.listeners[name](event); }
}
function setup(comparison = false) {
  const elements = new Map();
  const get = id => { if (!elements.has(id)) elements.set(id, new Element('div')); return elements.get(id); };
  get('filter').value = 'all';
  const samples = [0, 0.1, 0.2].map((time, i) => ({
    time_s: time, distance_m: time, speed_m_s: time, front_clearance_m: 0.9-time,
    encoder_counts: {left: i, right: i*2}, decoded_encoder_counts: {left: i*2, right: i*2},
    driver_pwm_duty: [0.2, 0.2], pack_current_a: 0.1, pack_voltage_v: 7.4, soc: 1,
  }));
  const evidence = {name: 'saved suite', source_sha256: 'digest', cases: [
    {name: '</script><unsafe>', status: 'failed', checks: [{name:'clearance',actual:0.7,expected:0.8,passed:false}], run: {
      samples, status:'completed_without_brake', brake_reason:null,
      metrics:{final_front_clearance_m:0.7,peak_sampled_pack_current_a:0.1,brake_command_time_s:null},
      events:[{time_s:0.15,kind:'fault',title:'counter freeze',detail:{count:2}}, {time_s:0.2,kind:'control',title:'brake',detail:{}}],
      configuration:{scenario:{}}, speed_control:{samples:[{time_s:0,target_speed_m_s:0,observation:null},{time_s:0.2,target_speed_m_s:0.2,observation:{mean_speed_m_s:0.19}}]},
    }},
    {name:'model_error',status:'error',checks:[],error:'traction limit',run:null},
  ]};
  const document = {
    getElementById:get, createElement:tag=>new Element(tag), createElementNS:(_,tag)=>new Element(tag),
    querySelectorAll:()=>get('cases').children,
  };
  if(comparison === true || comparison === 'power-comparison') {
    const candidate = JSON.parse(JSON.stringify(evidence.cases[0]));
    candidate.name='candidate / normal'; candidate.design='candidate'; candidate.peer_index=0; candidate.comparison_key='normal';
    candidate.run.samples.at(-1).time_s=0.3;
    evidence.cases[0].design='baseline'; evidence.cases[0].peer_index=1; evidence.cases[0].comparison_key='normal';
    evidence.cases[1]=candidate;
    evidence.comparison={design_changes:{total:1,changes:[{path:'/assembly/total_mass_kg',baseline:2.3,candidate:3.3}]},cases:[{
      name:'normal',baseline_status:'failed',candidate_status:'failed',baseline_outcome:'completed_without_brake',candidate_outcome:'completed_without_brake',
      metrics:{peak_sampled_pack_current_a:{unit:'A',baseline:0.1,candidate:0.2,delta:0.1}}
    }]};
  }
  if(comparison === 'bench' || comparison === 'bench-check') {
    evidence.cases=evidence.cases.slice(0,1);
    evidence.bench={source_kind:'synthetic',metadata:{instrument:'none',conditions:'fixed bias'},counts:{total:2,paired_rows:1},
      metrics:{speed_m_s:{unit:'m/s',paired_values:1,mean_error:-0.1,rmse:0.1,maximum_absolute_error:0.1}},
      rows:[{measurement_time_s:0.1,aligned_time_s:0.1,status:'aligned',values:{speed_m_s:{measured:1.1,predicted:1,error:-0.1}}},
        {measurement_time_s:0.5,aligned_time_s:0.5,status:'after_run',values:{}}]};
  }
  if(comparison === 'bench-check') {
    evidence.bench_acceptance={status:'fail',source_kind:'synthetic',policy:{name:'expected measured data'},coverage:{},limitations:[],checks:[{name:'declared_source_kind',status:'fail',actual:'synthetic',expected:'measured',unit:''},{name:'speed_m_s.maximum_rmse',status:'inconclusive',actual:null,expected:0.01,unit:'m/s'}]};
  }
  if(comparison === 'grade') evidence.cases[0].run.grade_rad=Math.PI/36;
  if(comparison === 'power' || comparison === 'power-comparison' || comparison === 'voltage-limit' || comparison === 'instant-voltage-limit' || comparison === 'power-response' || comparison === 'voltage-monitor' || comparison === 'voltage-errors') {
    for(const row of evidence.cases.filter(c=>c.run)) {
      row.run.power_conditions={initial_soc:0.5,auxiliary_current_a:0.25,vcc_v:5};
      row.run.samples.forEach(s=>{s.soc=0.5;s.auxiliary_current_a=0.25;});
    }
  }
  if(comparison === 'voltage-monitor' || comparison === 'voltage-errors') {
    const run=evidence.cases[0].run;
    run.schema_version=12;
    run.voltage_monitor={config:{resolution_bits:12,conversion_s:0.03},readings:[
      {observation:{sample_time_s:0,time_s:0.03,voltage_v:7.3,raw_count:2990,over_range:false}},
      {observation:{sample_time_s:0.1,time_s:0.13,voltage_v:7.2,raw_count:2949,over_range:false}}
    ],pending:{observation:{sample_time_s:0.19,time_s:0.22,voltage_v:7.1}}};
    run.events.push(...run.voltage_monitor.readings.map(c=>({time_s:c.observation.time_s,kind:'voltage',title:'Voltage conversion delivered',detail:c.observation})));
    if(comparison === 'voltage-errors') run.voltage_monitor.config.errors={gain:1.05,offset_v:-0.2,noise_bound_v:0.05,seed:7};
  }
  if(comparison === 'power-response') {
    const run=evidence.cases[0].run;
    run.schema_version=11;
    run.power_conditions.minimum_pack_voltage_v=7;
    run.power_conditions.voltage_limit_action='brake';
    run.power_limit_event={phase:'command_boundary',time_s:0.1,threshold_v:7,voltage_v:6.9};
    run.power_response={time_s:0.1,action:'brake'};
    run.status='stopped_after_power_limit';
    run.metrics.distance_after_power_limit_m=0.02;
    run.metrics.power_response_elapsed_s=0.1;
    run.samples.forEach(s=>{s.driver_modes=[s.time_s>0.1?'brake':'forward',s.time_s>0.1?'brake':'forward'];});
    run.samples.at(-1).interval_start={time_s:0.1,pack_power_w:0};
    run.samples.at(-1).speed_m_s=0.004;
    run.events.push({time_s:0.1,kind:'power',title:'Driver brake response latched',detail:run.power_response});
  }
  if(comparison === 'voltage-limit' || comparison === 'instant-voltage-limit') {
    const run=evidence.cases[0].run;
    run.power_conditions.minimum_pack_voltage_v=7;
    run.status='power_limit_reached';
    run.power_limit_event={phase:'command_boundary',time_s:0.2,threshold_v:7,voltage_v:6.9};
    run.events.push({time_s:0.2,kind:'power',title:'Declared voltage limit',detail:run.power_limit_event});
    if(comparison === 'instant-voltage-limit') {
      run.samples=run.samples.slice(0,1);
      run.power_limit_event.time_s=0;
      run.events=[{time_s:0,kind:'power',title:'Declared voltage limit',detail:run.power_limit_event}];
      run.metrics.final_front_clearance_m=0.9;
    }
  }
  const context=vm.createContext({document});
  vm.runInContext('const ROVER_EVIDENCE = '+JSON.stringify(evidence)+';\n'+script, context, {timeout:1000});
  get.evaluate=expression=>vm.runInContext(expression,context,{timeout:1000});
  return get;
}

test('initialization builds charts and renders report labels as text', () => {
  const get = setup();
  assert.equal(get('case-name').textContent, '</script><unsafe>');
  assert.equal(get('charts').children.length, 4);
  assert.equal(get('cases').children.length, 2);
  assert.equal(get('sample-index').textContent, '1 / 3 samples');
  assert.equal(get('checks').children[0].children[1].children.at(-1).textContent, 'FAIL');
});
test('power conditions label the initial charge and pack-side allowance and add charge and voltage plots', () => {
  const get=setup('power');
  assert.match(get('power-scope').textContent,/50.*%.*0\.2500 A pack-side.*5\.0+ V/);
  assert.equal(get('charts').children.length,6);
  assert.equal(get('charts').children[3].children[1].children.filter(e=>e.tag==='polyline').length,2);
});
test('power comparisons overlay the peer voltage and charge', () => {
  const get=setup('power-comparison');
  for(const index of [4,5]) {
    assert.equal(get('charts').children[index].children[1].children.filter(e=>e.tag==='polyline').length,2);
  }
});
test('voltage-limit view retains proposed-command context, threshold and power event evidence', () => {
  const get=setup('voltage-limit');
  assert.match(get('power-scope').textContent,/proposed motor command/);
  assert.ok(get('metrics').children.some(c=>c.children[0].textContent==='PROPOSED SUPPLY'));
  assert.equal(get('charts').children[4].children[1].children.filter(e=>e.tag==='polyline').length,2);
  get('filter').value='power';get('filter').fire('change');
  assert.equal(get('events').children.length,1);
  get('events').children[0].fire('click');
  assert.equal(get('time').textContent,'0.2000 s');
  assert.equal(JSON.parse(get('event-detail').textContent).detail.voltage_v,6.9);
});
test('instantaneous cutoff charts show saved points and do not label an invented future horizon', () => {
  const get=setup('instant-voltage-limit');
  assert.equal(get('sample-index').textContent,'1 / 1 samples');
  for(const box of get('charts').children) {
    const svg=box.children[1];
    assert.ok(svg.children.some(e=>e.tag==='circle'));
    assert.ok(svg.children.some(e=>e.tag==='text' && e.textContent==='single recorded instant'));
    assert.ok(svg.children.every(e=>Object.values(e.attributes).every(v=>!v.includes('NaN') && !v.includes('Infinity'))));
    svg.fire('click',{clientX:985});
    assert.equal(get('time').textContent,'0.0000 s');
  }
});

test('power-response view distinguishes a latched driver mode from the preceding interval', () => {
  const get=setup('power-response');
  assert.match(get('power-scope').textContent,/latched brake.*supplies retained/);
  assert.doesNotMatch(get('power-scope').textContent,/no.*stopped-state prediction/);
  assert.ok(get('metrics').children.some(c=>c.children[0].textContent==='DRIVER RESPONSE' && c.children[1].textContent==='brake'));
  assert.ok(get('metrics').children.some(c=>c.children[0].textContent==='TRAVEL AFTER LIMIT' && c.children[1].textContent==='0.0200 m'));
  assert.ok(get('metrics').children.some(c=>c.children[0].textContent==='FINAL SPEED' && c.children[1].textContent==='0.0040 m/s'));
  get('filter').value='power'; get('filter').fire('change');
  get('events').children[0].fire('click');
  assert.equal(get('time').textContent,'0.1000 s');
  assert.ok(get('readout').children.some(c=>c.textContent==='Driver L/R forward / forward'));
  assert.ok(get('readout').children.some(c=>c.textContent.startsWith('Boundary sample before the driver override')));
  get('playhead').value='2';get('playhead').fire('input');
  assert.ok(get('readout').children.some(c=>c.textContent==='Driver L/R brake / brake'));
  assert.ok(get('readout').children.some(c=>c.textContent==='Interval from 0.1000 s · starting pack power 0.0000 W'));
  assert.ok(!get('readout').children.some(c=>c.textContent.startsWith('Boundary sample before the driver override')));
});

test('voltage monitor shows delivered observations and keeps pending conversions out of the plot', () => {
  const get=setup('voltage-monitor');
  assert.match(get('power-scope').textContent,/ideal voltage monitor 12-bit.*0\.0300 s.*no MCU ADC/);
  assert.ok(get('metrics').children.some(c=>c.children[0].textContent==='CONVERSION AT RUN END'&&c.children[1].textContent==='pending'));
  const chart=get('charts').children[4];
  assert.ok(chart.children[0].children[0].children.some(c=>c.textContent==='monitor (delivered)'));
  assert.ok(get('readout').children.some(c=>c.textContent==='Voltage monitor: no delivered reading at this cursor'));
  get('playhead').value='1';get('playhead').fire('input');
  assert.ok(get('readout').children.some(c=>c.textContent==='Delivered voltage 7.3000 V · sampled at 0.0000 s'));
  get('playhead').value='2';get('playhead').fire('input');
  assert.ok(get('readout').children.some(c=>c.textContent==='Delivered voltage 7.2000 V · sampled at 0.1000 s'));
  get('filter').value='voltage';get('filter').fire('change');
  assert.equal(get('events').children.length,2);
  get('events').children[0].fire('click');
  assert.equal(JSON.parse(get('event-detail').textContent).detail.time_s,0.03);
});
test('voltage errors are labeled as declared assumptions alongside delivered readings', () => {
  const get=setup('voltage-errors');
  assert.match(get('power-scope').textContent,/voltage monitor with declared errors/);
  assert.match(get('power-scope').textContent,/gain 1\.0500.*offset -0\.2000 V.*noise ±0\.0500 V.*seed 7/);
  assert.ok(!get('power-scope').textContent.includes('ideal voltage monitor'));
  get('playhead').value='1';get('playhead').fire('input');
  assert.ok(get('readout').children.some(c=>c.textContent==='Delivered voltage 7.3000 V · sampled at 0.0000 s'));
});

test('uphill evidence labels the grade, lane coordinates and hill-holding limit', () => {
  const get=setup('grade');
  assert.match(get('fixture-scope').textContent,/5\.000° uphill/);
  assert.match(get('fixture-scope').textContent,/distances along the lane/);
  assert.match(get('fixture-scope').textContent,/does not establish hill holding/);
});
test('large plot envelopes retain single-sample peaks, valleys, endpoints and original time order', () => {
  const get=setup();
  const result=get.evaluate(`(() => {
    const points=Array.from({length:10001},(_,i)=>[i/1000,Math.sin(i)]);
    points[37][1]=999; points[38][1]=-999;
    const selected=envelopePoints(points);
    return {count:selected.length,first:selected[0][0],last:selected.at(-1)[0],
      peak:selected.some(p=>p===points[37]),valley:selected.some(p=>p===points[38]),
      ordered:selected.every((p,i)=>!i || selected[i-1][0]<p[0]),
      untouched:points.length===10001 && points[37][1]===999};
  })()`);
  assert.ok(result.count<=1500);
  assert.equal(result.first,0);assert.equal(result.last,10);
  assert.equal(result.peak,true);assert.equal(result.valley,true);
  assert.equal(result.ordered,true);assert.equal(result.untouched,true);
});
test('plot envelopes keep every small-series point and obey odd budgets for flat and changing data', () => {
  const get=setup();
  assert.equal(get.evaluate('(() => {const p=[[0,1],[1,2]];return envelopePoints(p)===p})()'),true);
  for(const limit of [4,5,11,1499,1500]) {
    assert.equal(get.evaluate(`(() => {const p=Array.from({length:5001},(_,i)=>[i,i%7]);const r=envelopePoints(p,${limit});return r.length<=${limit} && r[0]===p[0] && r.at(-1)===p.at(-1) && r.every((v,i)=>!i||r[i-1][0]<v[0])})()`),true);
  }
  assert.equal(get.evaluate('envelopePoints(Array.from({length:10000},(_,i)=>[i,1])).length<=1500'),true);
});
test('bench acceptance displays source mismatch and unavailable errors without claiming a pass', () => {
  const get=setup('bench-check');
  assert.equal(get('bench-acceptance').hidden,false);
  assert.match(get('bench-acceptance-status').textContent,/FAIL.*declared synthetic/);
  const rows=get('bench-acceptance-checks').children[0].children;
  assert.equal(rows[1].children[1].textContent,'FAIL');
  assert.equal(rows[2].children[1].textContent,'INCONCLUSIVE');
  assert.equal(rows[2].children[2].textContent,'unavailable');
});
test('slider uses exact saved sample and updates observed and decoded count readouts', () => {
  const get = setup();
  get('playhead').value='1'; get('playhead').fire('input');
  assert.equal(get('time').textContent, '0.1000 s');
  assert.ok(get('readout').children.some(e=>e.textContent==='Observed L/R 1 / 2'));
  assert.ok(get('readout').children.some(e=>e.textContent==='Decoded L/R 2 / 2'));
});
test('event selection chooses latest sample at or before event time', () => {
  const get = setup(); get('events').children[0].fire('click');
  assert.equal(get('time').textContent, '0.1000 s');
  assert.equal(JSON.parse(get('event-detail').textContent).time_s, 0.15);
});
test('event filter preserves actual event evidence', () => {
  const get = setup(); get('filter').value='control'; get('filter').fire('change');
  assert.equal(get('events').children.length,1);
  get('events').children[0].fire('click');
  assert.equal(get('time').textContent,'0.2000 s');
});
test('case errors hide the previous timeline and selecting a run resets its cursor', () => {
  const get = setup(); get('events').children[0].fire('click');
  get('cases').children[1].fire('click');
  assert.equal(get('run').hidden,true); assert.equal(get('error').textContent,'traction limit');
  get('cases').children[0].fire('click');
  assert.equal(get('run').hidden,false); assert.equal(get('time').textContent,'0.0000 s');
});
test('clicking the end of a plot selects the final saved sample', () => {
  const get = setup(); get('charts').children[0].children[1].fire('click',{clientX:985});
  assert.equal(get('time').textContent,'0.2000 s');
});
test('comparison shows metric deltas and overlays the peer physical speed', () => {
  const get=setup(true);
  assert.equal(get('comparison').hidden,false);
  const table=get('comparison-metrics').children[0];
  assert.equal(table.children[1].children[4].textContent,'0.1000');
  const svg=get('charts').children[0].children[1];
  assert.equal(svg.children.filter(e=>e.tag==='polyline').length,4);
  assert.ok(get('design-changes').textContent.includes('/assembly/total_mass_kg'));
});
test('peer traces do not invent samples beyond the selected runs end', () => {
  const get=setup(true);
  get('charts').children[0].children[1].fire('click',{clientX:985});
  assert.equal(get('time').textContent,'0.2000 s');
  get('cases').children[1].fire('click');
  get('charts').children[0].children[1].fire('click',{clientX:985});
  assert.equal(get('time').textContent,'0.3000 s');
});
test('bench view labels synthetic origin and overlays paired measurements only', () => {
  const get=setup('bench');
  assert.equal(get('bench').hidden,false);
  assert.ok(get('bench-source').textContent.includes('synthetic'));
  assert.ok(get('bench-source').textContent.includes('1/2 rows paired'));
  assert.equal(get('charts').children[0].children[1].children.filter(e=>e.tag==='polyline').length,4);
});
test('measurement row control moves only for in-range times and retains rejected evidence', () => {
  const get=setup('bench');
  get('bench-row').value='0'; get('bench-row').fire('input');
  assert.equal(get('time').textContent,'0.1000 s');
  get('bench-row').value='1'; get('bench-row').fire('input');
  assert.equal(get('time').textContent,'0.1000 s');
  assert.equal(JSON.parse(get('bench-detail').textContent).status,'after_run');
});
