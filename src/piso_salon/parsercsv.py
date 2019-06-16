import os
import pprint
import csv
"""
    Clase que lee un archivo cvs y devuelve una lista de diccionarios.

    El archivo cvs esta contenido con 3 parametros (x,y,probaOcupado)
    donde se da la probabilidad de que esté ocupado la cooordenada (x,y). 
    la probabilidad: está en un rango  de [0-100] 

    Para metodos graficos la primera coordenada esta definida la coordenada a donde 
    se tiene que llegar el robot. 
"""
class ParserCsv:
    def parse_file(self,file_name_csv):
        DATADIR = ""
        DATAFILE = file_name_csv
        datafile = os.path.join(DATADIR, DATAFILE)
        data = []
        with open(datafile, "rb") as f:
            r=csv.DictReader(f)
            for line in r:
                data.append(line)

        return data
    
if __name__ == "__main__":
    parser= ParserCsv()
    a=parser.parse_file("lugares_ocupados.csv")
    print(a)