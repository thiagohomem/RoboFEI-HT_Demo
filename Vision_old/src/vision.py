import numpy as np
import os
import cv2
import ctypes
import argparse
import time

from BallVision import *
from PanTilt import *

try:
    from configparser import ConfigParser
except ImportError:
    from ConfigParser import ConfigParser  # ver. < 3.0

parser = argparse.ArgumentParser(description='Robot Vision', epilog= 'Responsavel pela deteccao dos objetos em campo / Responsible for detection of Field objects')
parser.add_argument('--visionball', '--vb', action="store_true", help = 'Calibra valor para a visao da bola')
parser.add_argument('--withoutservo', '--ws', action="store_true", help = 'Servos desligado')
parser.add_argument('--head', '--he', action="store_true", help = 'Configurando parametros do controle da cabeca')

#----------------------------------------------------------------------------------------------------------------------------------

def statusBall(positionballframe):
	if positionballframe[0] == 0:
		print "Campo nao encontrada"
		
	if positionballframe[0] == 1:
		mens = "Bola nao encontrada, campo "
		
		if positionballframe[1] == -1:
			mens += "a esquerda"
		elif positionballframe[1] == 1:
			mens += "a direita"
		else:
			mens += "esta no meio"
		
		if positionballframe[2] == -1:
			mens += " cima"
		elif positionballframe[2] == 1:
			mens += " baixo"
		else:
			mens += " centro"
		print mens
	if positionballframe[0] == 2:
		print "Bola encontrada na posicao x: " + str(round(positionballframe[1],2)) + " y: " + str(round(positionballframe[2],2)) + " e raio de: " + str(round(positionballframe[3],2))

#----------------------------------------------------------------------------------------------------------------------------------

def readResolutions(Config):
	if 'Resolutions' not in Config.sections():
		print "Resolutions inexistente, criando valores padrao"
		Config.add_section('Resolutions')
	if len(Config.options('Resolutions')) == 0:
		Config.set('Resolutions','1','320x180')
		Config.set('Resolutions','2','640x480\n\t;Resolucoes nao pode ter valores iguais')
		with open('../Data/config.ini', 'wb') as configfile:
			Config.write(configfile)
		
	a = 0
	Resolutions = np.chararray((len(Config.options('Resolutions')), 2), itemsize=11)
	Resolutions[:]='0'
	for i in Config.options('Resolutions'):
			if '\n' in Config.get('Resolutions',i).rpartition('\n'):
				Resolutions[a,0] = Config.get('Resolutions',i).rpartition('\n')[0]
			else:
				Resolutions[a,0] = Config.get('Resolutions',i).rpartition('\n')[2]
			a += 1


	for a in range(0, len(Config.options('Resolutions'))):
			Resolutions[a,1] = Resolutions[a,0].rpartition('x')[2]
			Resolutions[a,0] = Resolutions[a,0].rpartition('x')[0]

	value = np.array( np.zeros( ( len( Config.options('Resolutions') ), 2 ), dtype=np.int ) )

	for i in range(0, len( Config.options('Resolutions') )):
			for j in range(0, 2):
				value[i,j] = int(Resolutions[i,j])
			
	return np.sort(value, axis=0)

#----------------------------------------------------------------------------------------------------------------------------------

def setResolution(status):
	global resolutions
	global atualres
	global dis
	global maxRadio
	
	if status[0] == 2:
#		print "Max: " + str(		maxRadio*pow(razao,atualres)		)
#		print "Min: " + str(		maxRadio*pow(razao,atualres+1)-2		)
		if status[3]>maxRadio*pow(razao,atualres):
			atualres -=1
			ret = cap.set(3,resolutions[atualres,0])
			ret = cap.set(4,resolutions[atualres,1])
		elif status[3]<maxRadio*pow(razao,atualres+1)-2:
			atualres +=1
			ret = cap.set(3,resolutions[atualres,0])
			ret = cap.set(4,resolutions[atualres,1])
	
	elif atualres != len(resolutions)/2 - 1:
		atualres = len(resolutions)/2 - 1
		ret = cap.set(3,resolutions[atualres,0])
		ret = cap.set(4,resolutions[atualres,1])

#----------------------------------------------------------------------------------------------------------------------------------
#Inicio programa

args = parser.parse_args()

ball = VisionBall(args)

if args.withoutservo == False:
	head = Pantilt(args, ball.Config)

minRadio = 0.8
maxRadio = 20.0

resolutions = readResolutions(ball.Config)
atualres = len(resolutions)/2 -1
razao = pow(minRadio/maxRadio, 1.0/len(resolutions))

cap = cv2.VideoCapture(0) #Abrindo camera

ret = cap.set(3,resolutions[atualres,0])
ret = cap.set(4,resolutions[atualres,1])

if args.withoutservo == False:
	posheadball = np.array([head.cen_posTILT,head.cen_posPAN]) #Iniciando valores iniciais da posicao da bola
os.system("v4l2-ctl -d /dev/video0 -c focus_auto=0 && v4l2-ctl -d /dev/video0 -c focus_absolute=0")

while True:
	#Salva o frame
	
	ret, frame = cap.read()
	
	positionballframe = ball.detect(frame,np.array([resolutions[atualres,0],resolutions[atualres,1]]))
	
	#status
	statusBall(positionballframe)
	
	if args.withoutservo == False:
		posheadball = head.mov(positionballframe,posheadball)
	
	setResolution(positionballframe)
	
	if args.visionball == True or args.head == True:
		print "Resolucao: " + str(resolutions[atualres,0]) + "x" + str(resolutions[atualres,1])
	
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
#	raw_input("Pressione enter pra continuar")

if args.withoutservo == False:
	head.finalize()
ball.finalize()
cv2.destroyAllWindows()
cap.release()
