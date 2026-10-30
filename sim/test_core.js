// track_sim 核心逻辑测试 (node)
const fs = require("fs");
const path = require("path");
const html = fs.readFileSync(path.join(__dirname, "track_sim.html"), "utf8");
const js = html.match(/<script>([\s\S]*?)<\/script>/)[1];

// 完整 mock DOM/Canvas
const fakeCtx = () => new Proxy({}, {
  get: (t, k) => (k === "canvas" ? {} : () => {}),
  set: () => true,
});
const fakeEl = () => ({
  textContent: "", value: "4", innerHTML: "", scrollTop: 0, scrollHeight: 0,
  style: {}, classList: { toggle: () => {}, add: () => {}, remove: () => {} },
  disabled: false, onclick: null, oninput: null, onchange: null,
  addEventListener: () => {}, getContext: () => fakeCtx(),
  clientWidth: 1200, clientHeight: 800,
});
const fakeCanvas = fakeEl();
global.document = { getElementById: (id) => (id === "cv" ? fakeCanvas : fakeEl()) };
global.window = { addEventListener: () => {} };
global.requestAnimationFrame = () => 0;

// 剥离 UI/渲染部分
let core = js.replace(/\/\* ================= UI =================[\s\S]*$/, "");
core = core.replace(/const cv=document\.getElementById\('cv'\), ctx=cv\.getContext\('2d'\);[\s\S]*?let drag=null;/, "");

const testBody = `
function logEv(){}
// --- 测试 ---
genTrack();
console.log("主线点数:", mainPts.length, " 十字索引:", crossIdx, " 六角入口:", hexEntryIdx);
console.log("车起点:", car.x.toFixed(0), car.y.toFixed(0), " th:", car.th.toFixed(2));
const n0=nearest(car.x,car.y,mainPts,true);
console.log("起点到主线距离:", n0.d.toFixed(1), " 主线点[10]:", mainPts[10].x.toFixed(0), mainPts[10].y.toFixed(0));
const r0=sensorValues();
console.log("起点 raw:", r0.map(v=>v.toFixed(3)).join(", "));
getError(r0);
console.log("起点归一化 L/ML/MR/R:", ADC.L, ADC.ML, ADC.MR, ADC.R, " 误差:", st.Error.toFixed(2));
let minE=1e9,maxE=-1e9,switched=0,lastState=st.TrackState,cross=0,hex=0,maxDist=0;
for(let i=0;i<3000;i++){
  step5ms();
  if(isNaN(car.x)||isNaN(car.y)||isNaN(car.th)){console.log("!!! NaN at step",i);break;}
  if(st.TrackState!==lastState){switched++;lastState=st.TrackState;}
  minE=Math.min(minE,st.Error);maxE=Math.max(maxE,st.Error);
  cross=Math.max(cross,st.CrossCount);hex=Math.max(hex,st.HexFlag);
  const n=nearest(car.x,car.y,mainPts,true);maxDist=Math.max(maxDist,n.d);
}
console.log("3000步(15s):");
console.log("  车:",car.x.toFixed(0),car.y.toFixed(0)," 速度:",car.v.toFixed(1),"mm/s");
console.log("  状态:",STATE_NAME[st.TrackState]," 切换:",switched," 误差:",minE.toFixed(1),"~",maxE.toFixed(1));
console.log("  十字计数:",cross," 六角Flag:",hex," 最大离线:",maxDist.toFixed(0),"mm");
const raw0=sensorValues();
console.log("  起点传感器:",raw0.map(v=>v.toFixed(4)).join(", "));
const states=new Set();
let hexEnter=0, prevHex=0, offFly=0, startX=car.x, startY=car.y, nearStart=0;
let maxSlow=0, maxFast=0, lastE=0, maxDE=0;
for(let i=0;i<40000;i++){
  step5ms();
  states.add(st.TrackState);
  if(st.HexFlag!==0 && prevHex===0) hexEnter++;
  prevHex=st.HexFlag;
  const n=nearest(car.x,car.y,mainPts,true);
  if(n.d>300) offFly++;
  // 回起点附近计数(粗略圈)
  if(Math.hypot(car.x-startX,car.y-startY)<80) nearStart++;
  maxSlow=Math.max(maxSlow,st.SFlSlow); maxFast=Math.max(maxFast,st.SFlFast);
  maxDE=Math.max(maxDE,Math.abs(st.Error-lastE)); lastE=st.Error;
}
console.log("  40000步(200s): 见到的状态:",[...states].map(s=>STATE_NAME[s]).join(", "));
console.log("  六角进入次数:",hexEnter," 离线>300mm 步数:",offFly," 回起点附近次数:",nearStart);
console.log("  反转计数峰值: SFlSlow=",maxSlow," SFlFast=",maxFast," 单步最大dError=",maxDE.toFixed(1));
console.log("  末位置:",car.x.toFixed(0),car.y.toFixed(0)," 末速度:",car.v.toFixed(0),"mm/s");
`;

eval(core + testBody);
