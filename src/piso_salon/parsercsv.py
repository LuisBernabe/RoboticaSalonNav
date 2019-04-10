import os
import pprint
import csv

class ParserCsv:
    def parse_file(file_name_csv):
        DATADIR = ""
        DATAFILE = file_name_csv
        datafile = os.path.join(DATADIR, DATAFILE)
        data = []
        with open(datafile, "rb") as f:
            r=csv.DictReader(f)
            for line in r:
                data.append(line)

        return data


    def test():
        DATADIR = ""
        DATAFILE = "lugares_ocupados.csv"
        datafile = os.path.join(DATADIR, DATAFILE)
        d = parse_file(datafile)
        print(d)

#test()
