// Copy to .build/build_slides.mjs before running with the bundled Node runtime.
import fs from 'node:fs/promises';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
import {Presentation, PresentationFile} from '@oai/artifact-tool';

const root=process.cwd(), out=path.join(root,'deliverables'), build=path.join(root,'.build');
process.env.RUNTIME_NODE_MODULES='C:/Users/PC/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules';
const skill='C:/Users/PC/.codex/plugins/cache/openai-primary-runtime/presentations/26.1013.11914/skills/presentations';
const {finalizePresentation,applyPresentationChartFont}=await import(pathToFileURL(path.join(skill,'container_tools/artifact_tool_utils.mjs')));
const metrics=JSON.parse(await fs.readFile(path.join(out,'metrics.json'),'utf8'));
const c=metrics.clean,n=metrics.noise;
const p=Presentation.create({slideSize:{width:1280,height:720}});
const font='Arial',ink='#15243A',muted='#526277',accent='#007D75',bg='#F6F8FA';
function text(s,t,x,y,w,h,size=25,color=ink,bold=false){
  const sh=s.shapes.add({geometry:'textbox',name:t.slice(0,45),position:{left:x,top:y,width:w,height:h},fill:'none',line:{fill:'none',width:0}});
  sh.text=t; sh.text.style={typeface:font,fontSize:size,color,bold};sh.text.insets=0;return sh;
}
function slide(title,sub){
  const s=p.slides.add();s.background.fill=bg;
  text(s,title,56,45,1168,70,43,ink,true);
  if(sub)text(s,sub,56,122,1168,45,22,muted);
  text(s,'VinSort · Track 4 · 16/10/2026',56,679,900,23,16,muted);
  text(s,`${p.slides.items.length}/6`,1174,679,50,23,16,muted);
  return s;
}
async function image(s,name,x,y,w,h){s.images.add({blob:new Uint8Array(await fs.readFile(path.join(out,'debug',name))),contentType:'image/png',position:{left:x,top:y,width:w,height:h},fit:'contain',alt:name});}
function note(s,t,source){s.speakerNotes.text=t+'\n\nNguồn: '+source;}
function box(s,t,x,y,w,h){s.shapes.add({geometry:'rect',position:{left:x,top:y,width:w,height:h},fill:'#E4EFED',line:{fill:accent,width:1},text:{text:t,style:{typeface:font,fontSize:23,color:ink}}});}
function arrow(s,x,y,w,h){s.shapes.add({geometry:'rightArrow',position:{left:x,top:y,width:w,height:h},fill:accent,line:{fill:'none',width:0}});}
function table(s,values,x,y,w,h,widths,size=22){
  const t=s.tables.add({rows:values.length,columns:values[0].length,left:x,top:y,width:w,height:h,columnWidths:widths,values});
  for(let r=0;r<values.length;r++)for(let col=0;col<values[0].length;col++){
    const cell=t.getCell(r,col);cell.fill=r===0?ink:(r%2?'#FFFFFF':'#EAF0F3');
    cell.text.style={typeface:font,fontSize:size,color:r===0?'#FFFFFF':ink,bold:r===0};
  }return t;
}

let s=slide('VinSort: phân loại linh kiện bằng thị giác','Vin Robotics × VinFast · Bối cảnh ứng dụng đề xuất');
text(s,'Tự động khi nhận rõ vật',56,208,515,90,42,accent,true);
text(s,'Chuyển cho người khi hai vật nhập nhằng',56,309,500,120,34,ink,true);
text(s,'Người dùng: kỹ thuật viên trạm phân loại\n\nMinh họa bằng 6 khối, 3 màu trong PyBullet',56,458,510,133,26,muted);
await image(s,'cover_camera.png',614,196,610,410);
note(s,'0:00–0:30, Lan. Phân loại linh kiện thủ công có thể nhầm. VinSort cho robot nhận biết vị trí và màu, đồng thời cho người can thiệp khi cảnh nhập nhằng. Đây là mô hình ứng dụng đề xuất, nhóm chưa có số liệu thực địa VinFast.','PHAN_CONG.md mục 7; vinsort/scene.py; ảnh camera ảo seed 0 từ lần chạy bàn giao.');

s=slide('Kiến trúc và trụ cột Track 4','Mã đã chạy: vinsort/ · Camera ảo nhìn vật, webcam thật chỉ nhìn tay');
box(s,'RGB + depth\nCamera ảo',56,208,220,94);arrow(s,292,240,40,25);
box(s,'HSV + contour\nPixel thành 3D',350,208,246,94);arrow(s,612,240,40,25);
box(s,'Kiểm tra tin cậy\narea / aspect / depth',670,208,260,94);arrow(s,946,240,40,25);
box(s,'AUTO\nIK + gắp + thùng',1004,208,220,94);
text(s,'Hình học: pixel + depth thành 3D',56,340,560,45,27,accent,true);
s.shapes.add({geometry:'downArrow',position:{left:692,top:314,width:32,height:86},fill:accent,line:{fill:'none',width:0}});
text(s,'suspicious',742,348,270,38,24,accent,true);
box(s,'Webcam / bàn phím\nEMA + pinch',56,416,285,94);arrow(s,357,448,42,25);
box(s,'TELEOP\nTách vật nhập nhằng',417,416,310,94);arrow(s,743,448,42,25);
box(s,'Phím a\nQuét lại và về AUTO',803,416,421,94);
text(s,'Bất định: suspicious gọi người vận hành, mất tay giữ vật\nThời gian: EMA 0,3 và giới hạn bước 4,5 cm/frame\nBằng chứng: seed cố định, đối chiếu ground truth của engine',56,551,1168,106,24,muted);
note(s,'0:30–1:00, Lan. Luồng trên là nhận thức và AUTO. Khi vùng ảnh không đủ tin cậy, điều phối chuyển TELEOP. Người tách vật rồi bấm a để robot nhìn lại. Pinch chỉ gắp/thả, không phải cử chỉ chuyển chế độ. Chuyển sang demo của A/B/C từ phút 1 đến phút 5.','vinsort/perception.py, controller.py, main.py, teleop.py; PHAN_CONG.md sơ đồ và trụ cột.');

s=slide('Số đo trong mô phỏng','20 seed thường, 120 vật · 5 seed lỗi · Đối chiếu ground truth PyBullet');
table(s,[['Chỉ số','Sạch','Nhiễu'],['Recall',`${c.matched}/${c.gt}`,`${n.matched}/${n.gt}`],['False positive',String(c.fp),String(n.fp)],['XY TB (mm)',c.mean.toFixed(2),n.mean.toFixed(2)],['Đúng thùng',`${c.correct}/${c.parts}`,`${n.correct}/${n.parts}`],['Cờ lỗi / báo nhầm',`${c.hits}/5 / ${c.flags}`,`${n.hits}/5 / ${n.flags}`]],56,192,598,316,[310,144,144],23);
const ch=s.charts.add('bar',{position:{left:682,top:190,width:542,height:322},title:'Sai số XY (mm)',titleTextStyle:{fontSize:24,fill:ink},categories:['Trung bình','Trung vị','Lớn nhất'],series:[{name:'Sạch',values:[c.mean,c.median,c.max].map(v=>Number(v.toFixed(2))),fill:accent},{name:'Nhiễu',values:[n.mean,n.median,n.max].map(v=>Number(v.toFixed(2))),fill:'#DD9B4F'}],barOptions:{direction:'column',grouping:'clustered'},hasLegend:true,legend:{position:'bottom',textStyle:{fontSize:19,fill:ink}},xAxis:{textStyle:{fontSize:18,fill:ink}},yAxis:{min:0,max:6,majorUnit:2,numberFormatCode:'0.0',textStyle:{fontSize:18,fill:ink}},dataLabels:{showValue:true,numberFormatCode:'0.00',position:'outEnd',textStyle:{fontSize:17,fill:ink}},chartFill:bg});
applyPresentationChartFont(ch,{fontFamily:font,availableFonts:[font]});
text(s,'Recall: ghép cùng màu, gần nhất XY trong 20 mm. FP: phát hiện không ghép được.\nXY: chuẩn Euclid trên cặp đã ghép. z: z phát hiện − (TABLE_Z + CUBE).\nĐúng thùng: tâm XY trong thùng đúng màu / mọi vật. Cờ lỗi: seed có suspicious.\nChu kỳ: tổng bước / 240 / lượt gắp thành công = '+c.cycle.toFixed(2)+' giây mô phỏng/vật.',56,535,1168,107,21,muted);
note(s,`5:00–5:40, Lan. Recall ${c.matched}/${c.gt}, XY trung bình ${c.mean.toFixed(2)} mm, đúng thùng ${c.correct}/${c.parts}. Biểu đồ tính trên các cặp của seed thường. Nhiễu nhân sáng U[0.7,1.3] cộng Gauss sigma 8, depth không đổi. Màu mô phỏng bão hòa nên kết quả nhiễu tương đương không chứng minh khả năng chịu ánh sáng thật. z trung bình ${c.z.toFixed(3)} mm. Chu kỳ là tham số mô phỏng. Các định nghĩa đầy đủ giữ trong REPORT.md và đầu benchmark.py.`,`deliverables/results/detect*.csv, pick*.csv, xy_pairs*.csv; metrics.json; benchmark.py. Nhiễu pick: 20 seed thường. Ca lỗi pick sạch: ${c.failure_correct}/${c.failure_parts} vật vào thùng, còn ${c.failure_remaining} vật.`);

s=slide('Ca lỗi: hai khối cùng màu sát nhau','Seed 100 · HSV tạo một vùng, contour gộp làm tâm phát hiện sai');
await image(s,'fail_rgb.png',56,198,373,280);await image(s,'fail_mask.png',453,198,373,280);await image(s,'fail_contour.png',850,198,373,280);
text(s,'RGB',56,483,373,34,24,accent,true);text(s,'Mask cùng màu',453,483,373,34,24,accent,true);text(s,'Contour gộp, cờ area',850,483,373,34,24,accent,true);
text(s,`Phản ứng: diện tích > 1,6 × 1510 px, gắn suspicious rồi chuyển TELEOP.\nPick sạch 5 seed lỗi: ${c.failure_correct}/${c.failure_parts} vật tự xếp, ${c.failure_remaining} vật cần can thiệp.\nKhắc phục: watershed hoặc cạnh depth. Kiểm thử có kịch bản phục hồi 6/6.`,56,538,1168,112,24,muted);
note(s,'5:40–6:10, Lan. Robot xếp bốn vật rõ trước rồi chuyển người. Hai vật gộp không phải gắp hụt. Người tách một vật ra rồi bấm a. Video và test_full_hybrid_story dùng tracker theo kịch bản, chưa kiểm chứng webcam. Nhóm có thể cải tiến tách vùng bằng watershed hoặc depth.','Ảnh camera ảo seed 100 trong deliverables/debug; vinsort/perception.py; test_vinsort.py; results/pick.csv.');

s=slide('Impact: kịch bản ước lượng','Các giả định dưới đây do nhóm đặt, chưa có nguồn đo tại VinFast');
table(s,[['Đại lượng','Giá trị','Căn cứ'],['Tỉ lệ đúng',`${c.correct}/${c.parts}`,'ĐO trong sim'],['Chu kỳ robot dự kiến','6 giây/vật','GIẢ ĐỊNH'],['Năng suất lý thuyết','3600 / 6 = 600 vật/giờ','ƯỚC LƯỢNG'],['Giờ vận hành trong phạm vi','1 ca × 8 giờ × 1 trạm','GIẢ ĐỊNH']],56,195,1168,300,[390,390,388],25);
text(s,`Đúng/giờ nếu đạt cùng tỉ lệ sim = 600 × ${(c.correct/c.parts).toFixed(2)} = ${(600*c.correct/c.parts).toFixed(0)}\nChưa trừ dừng máy, giám sát và thời gian can thiệp.`,56,523,1168,81,27,accent,true);
text(s,'Chưa có baseline người thật để tính ROI hoặc giờ công thực sự tiết kiệm.',56,622,1168,34,24,muted);
note(s,'6:10–6:35, Lan. Chỉ tỉ lệ đúng được đo trong sim. Chu kỳ sáu giây là giả định minh họa, không lấy chu kỳ mô phỏng làm năng suất thật. Tích tám giờ là giờ vận hành trong phạm vi tự động hóa, chưa phải tám giờ công tiết kiệm. Cần đo baseline và can thiệp.','REPORT.md mục Impact. 6 giây/vật, 1 ca, 8 giờ và 1 trạm là giả định của nhóm, không phải số liệu VinFast.');

s=slide('Giới hạn và đường sim-to-real','Phạm vi hiện tại: 3 màu bão hòa, camera và depth mô phỏng');
text(s,'Những gì cần kiểm chứng tiếp',56,199,1168,49,31,accent,true);
text(s,'Camera thật và ánh sáng nhà xưởng\nKim loại bóng, vật chồng và vật ngoài ba màu\nĐộ bám tay gắp thật, độ trễ và độ chính xác cử chỉ\nAn toàn vùng làm việc và dừng khẩn phần cứng',56,271,1168,193,30,ink);
text(s,'Lộ trình',56,495,1168,45,31,accent,true);
text(s,'Dữ liệu vật thật và baseline người  /  Hiệu chuẩn camera\nDetector phù hợp  /  Tay gắp thật  /  Pilot có đo can thiệp',56,555,1168,89,28,muted);
note(s,'6:35–7:00, Lan. Kết quả chứng minh pipeline và cơ chế bàn giao trong thế giới mô phỏng. Constraint chưa mô phỏng độ bám. Bước kế tiếp là dữ liệu và camera thật rồi tay gắp thật có đo số lần can thiệp. Từ phút 7 đến 10 cả nhóm hỏi đáp. Nếu demo hỏng, mở VinSort_demo.mp4 và nói rõ video mô phỏng, teleop theo kịch bản.','vinsort/controller.py (fixed constraint), perception.py (HSV), teleop.py, REPORT.md.');

await fs.mkdir(path.join(build,'previews'),{recursive:true});
for(let i=0;i<p.slides.items.length;i++){
 const b=await p.export({slide:p.slides.items[i],format:'png',scale:1});
 await fs.writeFile(path.join(build,'previews',`slide-${i+1}.png`),new Uint8Array(await b.arrayBuffer()));
}
const candidate=path.join(build,'candidate.pptx');
await(await PresentationFile.exportPptx(p)).save(candidate);
const final=path.join(out,'VinSort_6_slides.pptx');
console.log(await finalizePresentation({workspaceDir:root,candidatePath:candidate,finalPath:final,
 pythonExecutable:'C:/Users/PC/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe',
 integrityValidatorPath:path.join(skill,'container_tools/inspect_presentation_package_integrity.py'),
 layoutValidatorPath:path.join(skill,'container_tools/inspect_presentation_layout_geometry.py'),
 layoutArgs:['--expected-slide-size-emu','12192000,6858000','--validate-heading-fit','--require-native-table-slide','3','--require-native-table-slide','5'],
 explicitTotalSlideCount:6,requiredNativeTableOwnerSlides:[3,5],requiredNativeChartOwnerSlides:[3],
 materializeLiteralChartWorkbooks:true,fontPolicy:{basis:'design',families:[font]},verifyArtifactToolImport:true,
 receiptPath:path.join(build,'presentation.validation.json')}));
