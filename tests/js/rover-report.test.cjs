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
function setup() {
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
  vm.runInNewContext('const ROVER_EVIDENCE = '+JSON.stringify(evidence)+';\n'+script, {document}, {timeout:1000});
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
