import sys
import os.path
import csv

from utils import keydefaultdict


input_path = sys.argv[1]
input_filename = os.path.splitext(input_path)[0]

reader = csv.DictReader(open(input_path))


def make_csv_writer(gridname):
    'Uses input_filename and reader.fieldnames globals'
    path = f'{input_filename}_{gridname}.csv'
    writer = csv.DictWriter(open(path, 'w', newline=''), reader.fieldnames)
    writer.writeheader()
    return writer


gridname_to_csv_writer = keydefaultdict(make_csv_writer)

for row in reader:
    gridname_to_csv_writer[row['Grid Name']].writerow(row)
