import pprint
records = ['d,p,op,bw MB/s,overhead']
with open('eccost.log','r') as fileobj:

  lines = fileobj.readlines()
  for line in lines:
    if 'encode' in line:
      data_num = line.split('data_num:')[1].split('parity_num:')[0].strip()
      parity_num = line.split('parity_num:')[1].split(':')[0].strip()
      op = 'encode'
      bandwidth = line.split('sec =')[1].split('MB/s')[0].strip()
      overhead = float(parity_num) / ((float(data_num) + float(parity_num)))
      records.append('{},{},{},{},{}'.format(data_num,parity_num,op,bandwidth,overhead))

pprint.pprint(records)

with open('eccost.csv','w') as fileobj:
  for record in records:
    fileobj.write(record)
    fileobj.write('\n')
