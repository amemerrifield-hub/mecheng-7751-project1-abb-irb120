const fs = require('fs');
const { PDFParse } = require('pdf-parse');
(async () => {
  const buffer = fs.readFileSync(process.argv[2]);
  const parser = new PDFParse({ data: buffer });
  const result = await parser.getText();
  const text = (result.text || '').replace(/\s+/g,' ').trim();
  console.log(text.slice(0, 20000));
  fs.writeFileSync('./scripts/abb_spec_extracted.txt', result.text || '', 'utf8');
  await parser.destroy();
})();
