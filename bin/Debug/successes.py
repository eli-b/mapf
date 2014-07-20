import sys  
import os.path
import csv
from collections import Counter
from collections import defaultdict
from pprint import pprint
import numpy as np
import pylab

input_path = sys.argv[1]
input_filename = os.path.splitext(input_path)[0]
output_path = input_filename + ' with analysis' + '.csv'

reader = csv.DictReader(open(input_path, 'rb'))

target_fieldnames = [fieldname for fieldname in reader.fieldnames if fieldname.endswith('Success')]

successes = Counter()
successes_per_num_of_agents = defaultdict(Counter)
rows_per_num_of_agents = Counter()

for row in reader:
    #print row['Grid Name'], row['Num Of Agents'], row['Instance Id']
    try:
        solvers_that_succeeded = [col_name[:-len(" Success")] for col_name, col_val in row.iteritems() \
                            if col_name in target_fieldnames if int(col_val) != 0]
    except:
        for fieldname in target_fieldnames:
            print fieldname, ": ", row[fieldname]
        raise
    successes.update(solvers_that_succeeded)
    num_of_agents = int(row['Num Of Agents'])
    successes_per_num_of_agents[num_of_agents].update(solvers_that_succeeded)
    rows_per_num_of_agents[num_of_agents] += 1

print 'Overall success counts:'
pprint(sorted(successes.iteritems(), key=lambda x:x[1], reverse=True))

for num_of_agents in sorted(successes_per_num_of_agents):
    suc = successes_per_num_of_agents[num_of_agents]
    print 'Success count for %d agents' % (num_of_agents, )
    pprint(sorted(suc.iteritems(), key=lambda x:x[1], reverse=True))
    
successes_sorted_by_num_of_agents = sorted(successes_per_num_of_agents.iteritems())
data = []
for agents_num, counter in successes_sorted_by_num_of_agents:
    counts_sorted_by_alg_name = sorted(counter.iteritems())
    row = [agents_num]
    row.extend((float(count)/rows_per_num_of_agents[agents_num] for alg, count in counts_sorted_by_alg_name))
    data.append(row)
data = np.array(data)
x = data[:, 0]
for i, alg_name in enumerate(successes_sorted_by_num_of_agents[0][1].iterkeys()):
    pylab.plot(x, data[:, i+1], "D-", label=alg_name)
pylab.title('Success rates')
pylab.xlabel('Number of agents')
pylab.ylabel('Success rate')
pylab.yscale('linear')
pylab.ylim(0, 1)
pylab.legend(loc='lower left')
pylab.show()
