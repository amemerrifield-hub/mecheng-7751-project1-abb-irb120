const fs = require('fs');
const { PDFParse } = require('pdf-parse');
(async () => {
  const buffer = fs.readFileSync(process.argv[2]);
  const parser = new PDFParse({ data: buffer });
  const result = await parser.getText();
  const text = result.text || '';
  fs.writeFileSync('./scripts/robot_kinematics_extracted.txt', text, 'utf8');
  const condensed = text.replace(/\s+/g, ' ').trim();
  console.log(condensed.slice(0, 16000));
  await parser.destroy();
})();
