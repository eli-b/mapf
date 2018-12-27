import glob
import os
import csv

writer = csv.DictWriter(open('mdd-sat-dao.csv', 'wb'),
    ["Grid Name","Grid Rows","Grid Columns","Num Of Agents","Num Of Obstacles","Instance Id",
    'mdd-sat Success', 'mdd-sat Runtime', 'mdd-sat Solution Cost',
    'mdd-sat Generated (HL)', "mdd-sat Look Ahead Nodes Created (HL)",
    "mdd-sat Adoptions (HL)", "mdd-sat Nodes Expanded With Goal Cost (HL)",
    "mdd-sat Conflicts Bypassed With Adoption (HL)", "mdd-sat Expanded (HL)",
    ])
writer.writeheader()
for filename in glob.glob(r'C:\Users\Eli\Documents\Search\Guni\CPF-experiment\bin\Debug\mdd-sat dao\*'):
    try:
        #print(f'working on {filename}')
        with open(filename) as f:
            output = f.read()
        seconds = float(output.split()[-1])
        cost = int(output.splitlines()[1].split(':')[1])
        success = cost != 0
        if seconds < 50 and not success:
            print(f'failed fast for {filename}')
        rows = -1
        cols = -1
        obstacles = -1
        grid_name = filename.split('\\')[-1][:len('ost003d')]
        num_agents = int(filename.split('\\')[-1].split('-')[1])
        instance_id = int(filename.split('\\')[-1].split('-')[2].split('_')[0])
        writer.writerow({
            'mdd-sat Success': 1 if success else 0,
            'mdd-sat Solution Cost': cost if success else -2,
            'mdd-sat Runtime': seconds * 1000,
            'Num Of Agents': num_agents,
            'Instance Id': instance_id,
            'Grid Name': grid_name, 
            'Grid Rows': rows,
            'Grid Columns': cols,
            'Num Of Obstacles': obstacles,
            'mdd-sat Generated (HL)':-1, "mdd-sat Look Ahead Nodes Created (HL)":-1,
            "mdd-sat Adoptions (HL)":-1, "mdd-sat Nodes Expanded With Goal Cost (HL)":-1,
            "mdd-sat Conflicts Bypassed With Adoption (HL)":-1, "mdd-sat Expanded (HL)":-1,
        })
    except (ValueError, IndexError), e:
        print('problem with {filename}: {e}')
        print(f'content: {output}')
        print("delete? (y/n)")
        x = raw_input()
        if x == 'y':
            os.unlink(filename)
        continue
