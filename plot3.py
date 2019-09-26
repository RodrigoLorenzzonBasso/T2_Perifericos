import csv
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

temperatura = []
umidade = []
pressao = []
corrente = []
plots = 4
amostras = 0

with open('LOG.CSV') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    for row in readCSV:
        temperatura.append(row[1])
        umidade.append(row[2])
        pressao.append(row[3])
        corrente.append(row[4])
        amostras += 1

x = list(range(0,amostras,1))

numeros = [int(y) for y in temperatura]

ordenados = sorted(numeros)
#ordenados.append(1)
#ordenados.pop()

plt.subplot(2, 1, 1)
plt.yticks(ordenados)
plt.plot(x, temperatura)
plt.title('Grafico de Temperatura')
plt.ylabel('Temperatura (C)')

numeros = [int(x) for x in umidade]

plt.subplot(2,1,2)
plt.yticks(sorted(numeros))
plt.plot(x, umidade)
plt.title('Grafico de Umidade')
plt.ylabel('Umidade (%)')



plt.show()