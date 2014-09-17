import sys  
import os.path
import csv
from collections import Counter
from collections import defaultdict
from pprint import pprint
import numpy as np
import pylab

input_paths = sys.argv[1:]

readers = (csv.DictReader(open(input_path, 'rb')) for input_path in input_paths)

successes = Counter()
successes_per_num_of_agents = defaultdict(Counter)
rows_per_num_of_agents = Counter()

for i, reader in enumerate(readers):
    print "Reading input file ", i
    target_fieldnames = [fieldname for fieldname in reader.fieldnames if fieldname.endswith('Success')]
    for row in reader:
        #print row['Grid Name'], row['Num Of Agents'], row['Instance Id']
        try:
            solvers_that_succeeded = [col_name[:-len(" Success")] for col_name, col_val in row.iteritems() \
                                if col_name in target_fieldnames if int(col_val) != 0]
        except (TypeError, ValueError) as e:
            print "Problem in row: num agents=", row["Num Of Agents"], " instance id=", row["Instance Id"]
            raise
        successes.update(solvers_that_succeeded)
        num_of_agents = int(row['Num Of Agents'])
        successes_per_num_of_agents[num_of_agents].update(solvers_that_succeeded)
        rows_per_num_of_agents[num_of_agents] += 1

# Not printing overall success rates as the comparison might not be fair - the slower algs aren't run on the tougher problems
        
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
