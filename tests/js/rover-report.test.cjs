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
  if(comparison === true) {
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
