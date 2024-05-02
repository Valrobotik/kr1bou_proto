import serial

from hokuyo.driver import hokuyo
from hokuyo.tools import serial_port

import matplotlib.pyplot as plt
import math

import time

from sklearn.cluster import DBSCAN
import numpy as np


uart_port = '/dev/ttyACM0'
uart_speed = 19200


if __name__ == '__main__':

    laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
    port = serial_port.SerialPort(laser_serial)

    laser = hokuyo.Hokuyo(port)
    laser.laser_off()

    x = []
    y = []

    x_robot = 0.5
    y_robot = 0.5
    theta_robot = 0.0

    fig = plt.figure()
    plt.ion()  # Active le mode interactif
    laser.laser_off()
    laser.laser_on()
    while True:
        dict = laser.get_scan()
        x = []
        y = []
        for i in range(0,len(dict[0])):
            if(dict[1][i]>90):
                x_temp = dict[1][i]/1000*math.sin(math.radians(dict[0][i])-theta_robot+math.pi/2)+x_robot
                y_temp = dict[1][i]/1000*math.cos(math.radians(dict[0][i])-theta_robot+math.pi/2)+y_robot
                if x_temp>0 and x_temp<3 and y_temp>0 and y_temp<2:
                    x.append(x_temp)
                    y.append(y_temp)
        
        points = np.array([x, y]).T

        # Appliquer DBSCAN sur les points. eps est la distance maximale entre deux échantillons pour qu'ils soient considérés comme dans le même voisinage.
        try :
            db = DBSCAN(eps=0.06, min_samples=3).fit(points)

            labels = db.labels_

            # Pour chaque groupe, calculer le point moyen et l'ajouter à la liste des groupes.
            groups = []
            for group_id in set(labels):
                if group_id != -1:  # Ignorer le bruit
                    group_points = points[labels == group_id]
                    group_mean = group_points.mean(axis=0)
                    groups.append(group_mean)
        except:
            groups = []

        # Afficher les groupes

        plt.plot(x, y, '.', color='b')
        for group in groups:
            plt.plot(group[0], group[1], 'o', color='r')
            c1 = plt.Circle((group[0], group[1]), 0.2)
            plt.gca().add_artist(c1)

        plt.xlim((-0.1, 3.1))
        plt.ylim((-0.1, 2.1))
        plt.draw()  # Dessine la figure sans bloquer l'exécution
        plt.pause(0.01)  # Pause de 2 secondes
        plt.clf()  # Efface la figure pour le prochain tour de boucle
        

    plt.ioff()  # Désactive le mode interactif
    plt.show()