import csv
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# Ordem do csv
#i,temp,umid,press,amps,hora,min,seg,dia,mes,ano

temperatura = []
umidade = []
pressao = []
corrente = []
plots = 4

with open('LOG.CSV') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    for row in readCSV:
        temperatura.append(row[1])
        umidade.append(row[2])
        pressao.append(row[3])
        corrente.append(row[4])

amostras = np.arange(0,int(row[0]), 1) 

fig, axs = plt.subplots(plots)

axs[0].plot(amostras, temperatura)
axs[0].set(ylabel='Temperatura (c)',
       title='Grafico temperatura')

axs[1].plot(amostras, umidade)
axs[1].set(ylabel='Umidade (%)',
       title='Grafico umidade')

axs[2].plot(amostras, pressao)
axs[2].set(ylabel='pressao (hPa)',
       title='Grafico pressao')

axs[3].plot(amostras, corrente)
axs[3].set(ylabel='Corrente (A)',
       title='Grafico corrente')

plt.tight_layout()
axs[0].grid()
axs[1].grid()
axs[2].grid()
axs[3].grid()

fig.savefig("test.png")
plt.show()