#!/usr/bin/env python3

import os
import pprint
import csv

class ParserCsv:
    def parse_file(self,file_name_csv):
        #print(file_name_csv)

        DATADIR = os.path.dirname(os.path.abspath(__file__))
        DATAFILE = file_name_csv
        datafile = os.path.join(DATADIR, DATAFILE)
        data = []
        with open(datafile, "r") as f:
            r=csv.DictReader(f)
            for line in r:
                data.append(line)

        return data


    #def test(self):
    #    DATADIR = ""
    #    DATAFILE = "lugares_ocupados.csv"
    #    datafile = os.path.join(DATADIR, DATAFILE)
    #    d = parse_file(datafile)
    #    print(d)

#test()
#parser=ParserCsv()
#d=parser.parse_file("lugares_ocupados.csv")
#print(d)
