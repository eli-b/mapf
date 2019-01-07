import sys
import os.path
import csv

input_path = sys.argv[1]
input_filename = os.path.splitext(input_path)[0]
output_path = input_filename + ' with analysis' + '.csv'

reader = csv.DictReader(open(input_path))

runtime_fieldnames = [fieldname for fieldname in reader.fieldnames if fieldname.endswith('Runtime') and not fieldname.endswith('Average Runtime')]
writer_fieldnames = reader.fieldnames
for i in xrange(len(runtime_fieldnames)):
    writer_fieldnames += ["%d place" % (i + 1, )]

writer = csv.DictWriter(open(output_path, 'wb'), writer_fieldnames)
writer.writeheader()

for row in reader:
    solvers_and_runtimes = [(col_name[:-len(" Runtime")], float(col_val)) for col_name,col_val in row.iteritems() \
                            if col_name in runtime_fieldnames]
    solvers_and_runtimes.sort(key=lambda x:x[1])
    for i in xrange(len(runtime_fieldnames)):
        row["%d place" % (i + 1, )] = "%s runtime=%f speedup=%f" % (solvers_and_runtimes[i][0], solvers_and_runtimes[i][1], (solvers_and_runtimes[i + 1][1] / solvers_and_runtimes[i][1]) if (i + 1 < len(runtime_fieldnames)) else 1)
    writer.writerow(row)
