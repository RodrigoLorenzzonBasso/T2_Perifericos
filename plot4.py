import pandas as pd
import csv
import matplotlib.pyplot as plt

headers = ['i','temp','umid','press','amps','hora','min','seg','dia','mes','ano']

df = pd.read_csv('LOG.CSV',names=headers)

x = df['i']
y_temp = df['temp']
y_umid = df['umid']
y_press = df['press']
y_amps = df['amps']

plt.plot(x,y_temp)
plt.title('Grafico de Temperatura')
plt.ylabel('Temperatura (C)')
plt.figure()


plt.plot(x,y_umid)
plt.title('Grafico de Umidade')
plt.ylabel('Umidade (%)')
plt.figure()

plt.plot(x,y_press)
plt.title('Grafico de Pressao')
plt.ylabel('Pressao (hPa)')
plt.figure()

plt.plot(x,y_amps)
plt.title('Grafico de Corrente')
plt.ylabel('Amperes (mA)')
plt.show()