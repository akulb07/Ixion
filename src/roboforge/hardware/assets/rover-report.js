"use strict";
const el = id => document.getElementById(id);
const node = (tag, text, className) => {
  const n = document.createElement(tag);
  if (text !== undefined) n.textContent = text;
  if (className) n.className = className;
  return n;
};
const fmt = (value, unit = "", digits = 4) => value === null || value === undefined ? "unavailable" : `${Number(value).toFixed(digits)}${unit}`;
const colors = ["#b39aff", "#64c9d4", "#e7b970", "#ea91b2", "#91c9a2", "#c8b9e4"];
let active, peer = null, cursor = 0, plots = [];
const svgNS = "http://www.w3.org/2000/svg";
function svgNode(tag, attributes, text) {
  const n = document.createElementNS(svgNS, tag);
  for (const [key, value] of Object.entries(attributes)) n.setAttribute(key, value);
  if (text !== undefined) n.textContent = text;
  return n;
}
function envelopePoints(points, limit = 1500) {
  if(!Number.isInteger(limit) || limit < 4) throw new RangeError("Plot budget must be an integer of at least four");
  if(points.length <= limit) return points;
  const buckets=Math.floor((limit-2)/2), interior=points.length-2, selected=[points[0]];
  for(let bucket=0;bucket<buckets;bucket++) {
    const start=1+Math.floor(bucket*interior/buckets), end=1+Math.floor((bucket+1)*interior/buckets);
    let minimum=start, maximum=start;
    for(let index=start+1;index<end;index++) {
      if(points[index][1] < points[minimum][1]) minimum=index;
      if(points[index][1] > points[maximum][1]) maximum=index;
    }
    if(minimum===maximum) selected.push(points[minimum]);
    else if(minimum<maximum) selected.push(points[minimum],points[maximum]);
    else selected.push(points[maximum],points[minimum]);
  }
  selected.push(points.at(-1));
  return selected;
}
function chart(title, series) {
  const box = node("div", undefined, "chart");
  const heading = node("div", title, "chart-title"), legend = node("span", undefined, "legend");
  series.forEach((s, i) => {const label = node("span", s.name); label.style.setProperty("--series", colors[i]); legend.append(label);});
  heading.append(legend); box.append(heading);
  const svg = svgNode("svg", {viewBox:"0 0 1000 135", preserveAspectRatio:"none", role:"img", "aria-label":title});
  const extent = Math.max(active.samples.at(-1).time_s, peer?.samples.at(-1).time_s || 0);
  const end = extent || 1;
  let low = 0, high = 0;
  series.forEach(s => s.points.forEach(p => {low = Math.min(low, p[1]); high = Math.max(high, p[1]);}));
  if (low === high) high = low + 1;
  const scale = Math.max(Math.abs(low), Math.abs(high), Number.MIN_VALUE);
  const y = value => 106 - (value / scale - low / scale) / (high / scale - low / scale) * 88;
  for (const value of [low, low/2 + high/2, high]) {
    svg.append(svgNode("line", {x1:65, x2:985, y1:y(value), y2:y(value), stroke:"#252733"}));
    svg.append(svgNode("text", {x:0, y:y(value)+3}, value.toFixed(3)));
  }
  svg.append(svgNode("text", {x:65,y:128}, "0 s"), svgNode("text", {x:985,y:128,"text-anchor":"end"}, extent ? `${extent.toFixed(3)} s` : "single recorded instant"));
  series.forEach((s,i) => {
    const points = envelopePoints(s.points);
    svg.append(svgNode("polyline", {points:points.map(p => `${65+p[0]/end*920},${y(p[1])}`).join(" "), fill:"none", stroke:colors[i], "stroke-width":1.6}));
    if(points.length===1 || (points.length===2 && points[0][0]===points[1][0] && points[0][1]===points[1][1])) {
      svg.append(svgNode("circle",{cx:65+points[0][0]/end*920,cy:y(points[0][1]),r:3,fill:colors[i]}));
    }
  });
  const line = svgNode("line", {x1:65,x2:65,y1:12,y2:110,stroke:"#eee7ff","stroke-dasharray":"3 3"});
  svg.append(line); plots.push({line,end});
  svg.addEventListener("click", event => {
    const rect = svg.getBoundingClientRect();
    seek(Math.max(0, Math.min(end, ((event.clientX-rect.left)/rect.width*1000-65)/920*end)));
  });
  box.append(svg); el("charts").append(box);
}
function seek(time) {
  let low=0, high=active.samples.length-1;
  while (low < high) {const middle = Math.ceil((low+high)/2); if(active.samples[middle].time_s <= time) low=middle; else high=middle-1;}
  cursor=low; el("playhead").value=String(cursor); inspect();
}
function inspect() {
  const sample = active.samples[cursor];
  const voltageReading=active.voltage_monitor?.readings.findLast(c=>c.observation.time_s<=sample.time_s)?.observation;
  el("time").textContent=fmt(sample.time_s," s");
  el("sample-index").textContent=`${cursor+1} / ${active.samples.length} samples`;
  el("readout").replaceChildren(...[
    `Travel ${fmt(sample.distance_m," m")}`, `Speed ${fmt(sample.speed_m_s," m/s")}`,
    `Clearance ${fmt(sample.front_clearance_m," m")}`, `Pack ${fmt(sample.pack_voltage_v," V",3)}`,
    `Charge ${fmt(sample.soc*100,"%",2)}`, `Observed L/R ${sample.encoder_counts.left} / ${sample.encoder_counts.right}`,
    `Decoded L/R ${sample.decoded_encoder_counts.left} / ${sample.decoded_encoder_counts.right}`,
    ...(sample.driver_modes ? [`Driver L/R ${sample.driver_modes.join(" / ")}`] : []),
    ...(sample.interval_start ? [`Interval from ${fmt(sample.interval_start.time_s," s")} · starting pack power ${fmt(sample.interval_start.pack_power_w," W")}`] : []),
    ...(active.power_response && sample.time_s===active.power_response.time_s ? ["Boundary sample before the driver override; replacement command is in the response event"] : []),
    ...(active.voltage_monitor ? [voltageReading ? `Delivered voltage ${fmt(voltageReading.voltage_v," V")} · sampled at ${fmt(voltageReading.sample_time_s," s")}${voltageReading.over_range ? " · OVER RANGE" : ""}` : "Voltage monitor: no delivered reading at this cursor"] : [])
  ].map(text => node("span",text)));
  plots.forEach(p => {const x=65+sample.time_s/p.end*920; p.line.setAttribute("x1",x); p.line.setAttribute("x2",x);});
}
function showEvents() {
  const list=el("events"); list.replaceChildren();
  const events=active.events.filter(e => el("filter").value === "all" || e.kind === el("filter").value);
  if(!events.length) list.append(node("p","No recorded events of this type."));
  for(const event of events) {
    const button=node("button",`${fmt(event.time_s," s")}  ${event.title}`);
    button.addEventListener("click",() => {seek(event.time_s); el("event-detail").textContent=JSON.stringify(event,null,2);});
    list.append(button);
  }
}
function showComparison(selected) {
  const comparison=ROVER_EVIDENCE.comparison;
  el("comparison").hidden=!comparison;
  if(!comparison) return;
  const result=comparison.cases.find(c => c.name===selected.comparison_key);
  el("comparison-outcomes").textContent=`Baseline: ${result.baseline_status} / ${result.baseline_outcome || "model error"}. Candidate: ${result.candidate_status} / ${result.candidate_outcome || "model error"}. Candidate acceptance is the gate; deltas are not a better/worse score.`;
  const table=node("table"), header=node("tr");
  ["METRIC","UNIT","BASELINE","CANDIDATE","DELTA"].forEach(label => header.append(node("th",label))); table.append(header);
  for(const [name,metric] of Object.entries(result.metrics)) {
    const row=node("tr"); [name,metric.unit,fmt(metric.baseline),fmt(metric.candidate),fmt(metric.delta)].forEach(text => row.append(node("td",text)));
    table.append(row);
  }
  el("comparison-metrics").replaceChildren(table);
  el("design-changes").textContent=JSON.stringify(comparison.design_changes,null,2);
}
function showBench() {
  const bench=ROVER_EVIDENCE.bench;
  el("bench").hidden=!bench;
  const acceptance=ROVER_EVIDENCE.bench_acceptance;
  el("bench-acceptance").hidden=!acceptance;
  if(acceptance) {
    el("bench-acceptance-status").textContent=`${acceptance.status.toUpperCase()} · ${acceptance.policy.name} · declared ${acceptance.source_kind} evidence. Passing these limits does not establish engineering readiness.`;
    const table=node("table"), header=node("tr");
    ["CHECK","RESULT","ACTUAL","LIMIT","UNIT"].forEach(t=>header.append(node("th",t)));table.append(header);
    for(const check of acceptance.checks) {
      const row=node("tr");[check.name,check.status.toUpperCase(),check.actual === null ? "unavailable" : String(check.actual),String(check.expected),check.unit].forEach(t=>row.append(node("td",t)));table.append(row);
    }
    el("bench-acceptance-checks").replaceChildren(table);
    el("bench-acceptance-policy").textContent=JSON.stringify({policy:acceptance.policy,coverage:acceptance.coverage,limitations:acceptance.limitations},null,2);
  }
  if(!bench) return;
  el("bench-source").textContent=`Declared ${bench.source_kind} data. ${bench.counts.paired_rows}/${bench.counts.total} rows paired. Instrument: ${bench.metadata.instrument}. Conditions: ${bench.metadata.conditions}. This is a discrepancy calculation, not a hardware accuracy certificate.`;
  const table=node("table"), header=node("tr");
  ["SIGNAL","UNIT","PAIRS","MEAN ERROR","RMSE","MAX ABS ERROR"].forEach(t=>header.append(node("th",t)));table.append(header);
  for(const [name,m] of Object.entries(bench.metrics)) {
    const row=node("tr"); [name,m.unit,String(m.paired_values),fmt(m.mean_error),fmt(m.rmse),fmt(m.maximum_absolute_error)].forEach(t=>row.append(node("td",t)));table.append(row);
  }
  el("bench-metrics").replaceChildren(table);
  el("bench-row").max=String(bench.rows.length-1);el("bench-row").value="0";
  inspectBenchRow(false);
}
function inspectBenchRow(moveCursor=true) {
  const bench=ROVER_EVIDENCE.bench;
  if(!bench) return;
  const index=Number(el("bench-row").value), row=bench.rows[index];
  el("bench-row-label").textContent=`${index+1}/${bench.rows.length} · ${row.status}`;
  el("bench-detail").textContent=JSON.stringify(row,null,2);
  if(moveCursor && row.aligned_time_s >= active.samples[0].time_s && row.aligned_time_s <= active.samples.at(-1).time_s) seek(row.aligned_time_s);
}
function benchSeries(signal) {
  const bench=ROVER_EVIDENCE.bench;
  if(!bench) return null;
  const points=bench.rows.filter(row=>row.status==="aligned" && row.values[signal]).map(row=>[row.aligned_time_s,row.values[signal].measured]);
  return points.length ? {name:`${bench.source_kind} (declared)`,points} : null;
}
function selectCase(index) {
  const selected=ROVER_EVIDENCE.cases[index]; active=selected.run; cursor=0; plots=[];
  peer=selected.peer_index === undefined ? null : ROVER_EVIDENCE.cases[selected.peer_index].run;
  document.querySelectorAll("#cases button").forEach((button,i) => button.setAttribute("aria-current",String(i===index)));
  el("case-name").textContent=selected.name;
  el("outcome").textContent=active ? `${selected.status} / ${active.status}` : selected.status;
  el("error").textContent=selected.error || ""; el("run").hidden=!active;
  showComparison(selected);
  showBench();
  if(!active) return;
  const grade=active.grade_rad ?? 0;
  el("fixture-scope").textContent=grade === 0 ? "FIXTURE · level straight travel · distances along the lane" : `FIXTURE · ${fmt(grade*180/Math.PI,"°",3)} uphill · distances along the lane · stop threshold does not establish hill holding`;
  const power=active.power_conditions ?? {initial_soc:1,auxiliary_current_a:0,vcc_v:3.3};
  el("power-scope").textContent=`POWER · initial SOC ${fmt(power.initial_soc*100,"%",2)} · auxiliary ${fmt(power.auxiliary_current_a," A pack-side")} · driver VCC ${fmt(power.vcc_v," V")} · regulator and BMS behavior unmodeled`;
  if(power.minimum_pack_voltage_v !== undefined) el("power-scope").textContent+=` · declared voltage limit ${fmt(power.minimum_pack_voltage_v," V")} · `+(power.voltage_limit_action ? `latched ${power.voltage_limit_action} driver response; pack and logic supplies retained` : "run boundary, no disconnection or stopped-state prediction");
  if(active.power_limit_event?.phase === "command_boundary") el("power-scope").textContent+=" · proposed motor command checked before the next motion interval";
  if(active.voltage_monitor) {
    const monitor=active.voltage_monitor.config, errors=monitor.errors;
    el("power-scope").textContent+=` · ${errors ? "voltage monitor with declared errors" : "ideal voltage monitor"} ${monitor.resolution_bits}-bit · conversion ${fmt(monitor.conversion_s," s")} · no MCU ADC circuit model`;
    if(errors) el("power-scope").textContent+=` · gain ${fmt(errors.gain)} · offset ${fmt(errors.offset_v," V")} · noise ±${fmt(errors.noise_bound_v," V")} · seed ${errors.seed}`;
  }
  el("metrics").replaceChildren(...[
    ["FINAL CLEARANCE",fmt(active.metrics.final_front_clearance_m," m")],
    ["BRAKE COMMAND",fmt(active.metrics.brake_command_time_s," s")],
    ["PEAK PACK CURRENT",fmt(active.metrics.peak_sampled_pack_current_a," A")],
    ["BRAKE REASON",active.brake_reason || "none"],
    ...(power.minimum_pack_voltage_v === undefined ? [] : [["POWER LIMIT",active.power_limit_event ? fmt(active.power_limit_event.time_s," s") : "not reached"]]),
    ...(active.power_limit_event ? [[active.power_limit_event.phase === "command_boundary" ? "PROPOSED SUPPLY" : "BOUNDARY VOLTAGE",fmt(active.power_limit_event.voltage_v," V")]] : []),
    ...(power.voltage_limit_action ? [["DRIVER RESPONSE",active.power_response?.action || "not triggered"]] : []),
    ...(active.power_response ? [["TRAVEL AFTER LIMIT",fmt(active.metrics.distance_after_power_limit_m," m")],["RESPONSE ELAPSED",fmt(active.metrics.power_response_elapsed_s," s")],["FINAL SPEED",fmt(active.samples.at(-1).speed_m_s," m/s")]] : []),
    ...(active.voltage_monitor ? [["VOLTAGE READINGS",String(active.voltage_monitor.readings.length)],["CONVERSION AT RUN END",active.voltage_monitor.pending ? "pending" : "none"]] : [])
  ].map(([label,value]) => {const card=node("div",undefined,"metric");card.append(node("span",label),node("strong",value));return card;}));
  el("playhead").max=String(active.samples.length-1); el("playhead").value="0";
  el("charts").replaceChildren();
  const sampled=(name,get) => ({name,points:active.samples.map(s => [s.time_s,get(s)])});
  const speed=[sampled("physical",s => s.speed_m_s)];
  const trace=active.speed_control?.samples || [];
  if(trace.length) {
    speed.push({name:"observed mean",points:trace.filter(s => s.observation).map(s => [s.time_s,s.observation.mean_speed_m_s])});
    speed.push({name:"target",points:trace.map(s => [s.time_s,s.target_speed_m_s])});
  }
  const peerLabel=selected.design === "baseline" ? "candidate" : "baseline";
  if(peer) speed.push({name:`${peerLabel} physical`,points:peer.samples.map(s => [s.time_s,s.speed_m_s])});
  const benchSpeed=benchSeries("speed_m_s"); if(benchSpeed) speed.push(benchSpeed);
  chart("WHEEL SPEED · m/s",speed);
  chart("ENCODER COUNTS",[
    sampled("observed L",s => s.encoder_counts.left),sampled("observed R",s => s.encoder_counts.right),
    sampled("decoded L",s => s.decoded_encoder_counts.left),sampled("decoded R",s => s.decoded_encoder_counts.right)
  ]);
  const pwm=[sampled("left",s => s.driver_pwm_duty[0]),sampled("right",s => s.driver_pwm_duty[1])];
  const current=[sampled("pack",s => s.pack_current_a)];
  if(active.power_conditions) current.push(sampled("auxiliary",s=>s.auxiliary_current_a));
  if(peer) {
    pwm.push({name:`${peerLabel} left`,points:peer.samples.map(s => [s.time_s,s.driver_pwm_duty[0]])});
    pwm.push({name:`${peerLabel} right`,points:peer.samples.map(s => [s.time_s,s.driver_pwm_duty[1]])});
    current.push({name:`${peerLabel} pack`,points:peer.samples.map(s => [s.time_s,s.pack_current_a])});
  }
  const benchCurrent=benchSeries("pack_current_a"); if(benchCurrent) current.push(benchCurrent);
  chart("DRIVER PWM · duty",pwm); chart("BATTERY CURRENT · A",current);
  for(const [field,title] of [["pack_voltage_v","BATTERY VOLTAGE · V"],["distance_m","TRAVEL · m"]]) {
    const measured=benchSeries(field);
    if(measured || (active.power_conditions && field==="pack_voltage_v")) {
      const series=[sampled("simulated",s=>s[field])];
      if(peer) series.push({name:`${peerLabel} simulated`,points:peer.samples.map(s=>[s.time_s,s[field]])});
      if(measured) series.push(measured);
      if(field==="pack_voltage_v" && active.voltage_monitor) series.push({name:"monitor (delivered)",points:active.voltage_monitor.readings.map(c=>[c.observation.time_s,c.observation.voltage_v])});
      if(field==="pack_voltage_v" && peer?.voltage_monitor) series.push({name:`${peerLabel} monitor (delivered)`,points:peer.voltage_monitor.readings.map(c=>[c.observation.time_s,c.observation.voltage_v])});
      if(field==="pack_voltage_v" && power.minimum_pack_voltage_v !== undefined) series.push({name:"declared voltage limit",points:[[active.samples[0].time_s,power.minimum_pack_voltage_v],[active.samples.at(-1).time_s,power.minimum_pack_voltage_v]]});
      chart(title,series);
    }
  }
  if(active.power_conditions) {
    const charge=[sampled("SOC",s=>100*s.soc)];
    if(peer) charge.push({name:`${peerLabel} SOC`,points:peer.samples.map(s=>[s.time_s,100*s.soc])});
    chart("BATTERY CHARGE · %",charge);
  }
  el("checks").replaceChildren();
  if(!selected.checks.length) el("checks").append(node("p","No acceptance policy is attached to this individual run."));
  else {
    const table=node("table"), head=node("tr");
    ["CHECK","ACTUAL","EXPECTED","RESULT"].forEach(t => head.append(node("th",t)));table.append(head);
    selected.checks.forEach(c => {const row=node("tr");[c.name,JSON.stringify(c.actual),JSON.stringify(c.expected)].forEach(t => row.append(node("td",t)));row.append(node("td",c.passed?"PASS":"FAIL",c.passed?"pass":"fail"));table.append(row);});
    el("checks").append(table);
  }
  el("event-detail").textContent="Select an event for saved evidence.";
  el("configuration").textContent=JSON.stringify(active.configuration,null,2); showEvents(); inspect();
}
el("suite-name").textContent=ROVER_EVIDENCE.name;
el("fingerprint").textContent=`Source JSON SHA-256: ${ROVER_EVIDENCE.source_sha256}`;
ROVER_EVIDENCE.cases.forEach((c,index) => {const button=node("button",c.name);button.append(node("small",c.status));button.addEventListener("click",() => selectCase(index));el("cases").append(button);});
el("playhead").addEventListener("input",() => {cursor=Number(el("playhead").value);inspect();});
el("filter").addEventListener("change",showEvents);
el("bench-row").addEventListener("input",()=>inspectBenchRow());
selectCase(0);
