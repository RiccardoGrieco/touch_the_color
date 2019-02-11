#!/usr/bin/env python

import rospy
import time
import random
import math

from sys import maxint
from numpy import pi
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class PotentialFieldsWander:
	def __init__(self):
		rospy.init_node("potentialFieldsss", anonymous=True)
		self.velPub = rospy.Publisher("/RosAria/cmd_vel", Twist, queue_size = 1)
		self.sonarSub = rospy.Subscriber("/RosAria/sonar", PointCloud, self.readSonar)
		self.poseSub = rospy.Subscriber("/RosAria/pose", Odometry, self.updatePose)
		self.sonarReading = PointCloud()
		self.RAD = pi/180
		self.releaser = False
		self.pose = Odometry()
		self.distance = [10 for i in range(0, 16)]		
		self.iter = 0
		self.readIter = maxint
		self.canRead = True
		self.fieldsPoints = list()

		# calcolo approssimativo dell'angolo fra il "di fronte" e il sonar i-esimo
		self.sonarAngles =  [i for i in range(0,16)]
		for i in range(0, 4):
			self.sonarAngles[i] = (-11.25 - (3 - i) * 22.5) * self.RAD
		for i in range(4, 12):
			self.sonarAngles[i] = (11.25 + (i - 4) * 22.5) * self.RAD
		for i in range(12, 16):
			self.sonarAngles[i] = (-180 + 11.25 + (i - 12) * 22.5) * self.RAD

	def readSonar(self, msg):
		self.sonarReading = msg
		i = 0

		if self.canRead:
			del self.fieldsPoints[:]
			# print("-----------------------------------------")
			for p in self.sonarReading.points:
				# if 0 < i <= 6:
				# print("sto leggendo i sensori da 1 a 6.")
				self.distance[i] = math.sqrt(p.x * p.x + p.y * p.y)

				if self.distance[i] <= 5:
					# consideriamo le letture fino a 5mt
					if self.distance[i] <= 1:
						if self.distance[i] == 0:
							self.distance[i] = 0.001
						self.fieldsPoints.append([(6-self.distance[i]) * (-p.x / self.distance[i]), (6-self.distance[i]) * (-p.y / self.distance[i])])
						#self.fieldsPoints.append([1/(self.distance[i]**3) * (-p.x/self.distance[i]), (1/self.distance[i]**3) * (-p.y/self.distance[i])]) # TODO: trasformare il vettore lettura
						# self.fieldsPoints.append([(-1/(p.x**3)), (-1/(p.y**3))]) # TODO: trasformare il vettore lettura
						if self.distance[i] < 0.5:
							# attiva releaser di avoid
							self.releaser = True
							# print("PointCloud sotto la soglia:")
							# print(p)
						
							# print("Lettura Sonar che ha fatto scattare il releaser:", i)
							# disabilito la lettura per tre secondi
							# print("indice ostacolo: ", i)
							# self.obstacleIndex = i # indice direzione primo ostacolo
			i = i + 1

	def updatePose(self, msg):
		self.pose = msg

	def wander(self):
		# spostamento casuale (a campi di potenziale)
		if self.canRead:
			angle = random.uniform(-90 * self.RAD, 90 * self.RAD)
		else:
			angle = 0
		return [4, 0]      # [x, y]

	def avoid(self):
		maxWalkableDistance = 5
		force = [0, 0]
		for p in self.fieldsPoints:
			force = [force[0] + p[0], force[1] + p[1]]
		return force

	# def rotate(self, velocity, degree):
	# 	direction = Twist()
	# 	percorso = 0
	# 	msgQuaternion = self.pose.pose.pose.orientation
	# 	startA = euler_from_quaternion([msgQuaternion.x, msgQuaternion.y, msgQuaternion.z, msgQuaternion.w])[2]

	# 	if startA * degree > 0 and abs(startA + degree) >= pi:
	# 		if degree < 0:
	# 			degree = 2*pi + degree
	# 		else:
	# 			degree = 2*pi - degree
	# 	#print("degree: ", degree)
		
	# 	if degree < 0:
	# 		velocity = - velocity
	# 		degree = - degree
	# 	direction.angular.z = velocity
		
	# 	while percorso < degree:
	# 		# t = percorso/self.direction.angular.z
	# 		time.sleep(0.5)
	# 		msgQuaternion = self.pose.pose.pose.orientation
	# 		percorso = abs(euler_from_quaternion([msgQuaternion.x, msgQuaternion.y, msgQuaternion.z, msgQuaternion.w])[2] - startA)
	# 		#percorso = abs(self.pose.pose.pose.orientation.z - startA)
	# 		self.velPub.publish(direction)
	# 		#print("percorso: ", percorso)
	# 	self.velPub.publish(Twist())

	def rotate(self, velocity, degree):
		direction = Twist()
		if degree < 0:
			velocity = - velocity
			degree = - degree
		direction.angular.z = velocity
		t0 = rospy.Time.now().to_sec()
		current_angle = 0

		while(current_angle < degree):
			self.velPub.publish(direction)
			t1 = rospy.Time.now().to_sec()
			current_angle = abs(velocity*(t1-t0))
		
		self.velPub.publish(Twist())	# per fermarlo

	def start(self):
		rate_start = rospy.Rate(1)
		rate_start.sleep() # attesa dell'inizializzazione dei nodi di ros
		# control loop
		while True:
			print("Control Loop: ", self.iter)
			self.iter += 1	
			if len(self.sonarReading.points) != 0:
				#print(type(self.sonarReading.points[0]))
				#print(self.sonarReading.points[0])

				rotation = self.wander()
				
				# sussunzione
				if self.releaser:
					# sommiamo wander a avoid per cercare di seguire la sua direzione
					print("Releaser acceso!")
					tmp = self.avoid()
					rotation[0] = rotation[0] + tmp[0]
					rotation[1] = rotation[1] + tmp[1]
					self.releaser = False
				
				self.move2(rotation)
			time.sleep(1)

	def move(self, force):
		angularTreshold = 15 * self.RAD	# definisce il range
		angularVelocity = 0.3
		# print("Angolo: ", rotation)
		self.canRead = False
		degree = math.atan2(force[1], force[0])# - pi/2
		print("degree dopo atan2: ", degree)
		if -angularTreshold < degree < angularTreshold:
			intensity = min(5, math.sqrt(force[0]**2 + force[1]**2))	# norma euclidea, prendiamo al max dist = 5
			intensity = 0.1 + (0.2 - 0.1) * (intensity/5)	# min + (max spostamento nell'intervallo) * (intensita' normalizzata)
			self.moveForward(intensity)
		else:
			self.rotate(angularVelocity, degree)

		self.canRead = True


	def move2(self, force):
		# facciamo un movimento solo...
		angularTreshold = 15 * self.RAD	# definisce il range
		angularVelocity = 0.3
		# print("Angolo: ", rotation)
		self.canRead = False
		degree = math.atan2(force[1], force[0])# - pi/2
		msg = Twist()
		# print("degree dopo atan2: ", degree)

		#intensity = min(5, abs(force[0]))
		if force[0] < 0:
			print("sono entrato nell'if con intensity: ", force[0])
			intensity = max(-5, force[0])
			sign = -1
		else:
			print("sono entrato nell'if con intensity: ", force[0])
			intensity = min(5, force[0])
			sign = 1
		print("intensity dopo: ", intensity)
		intensity = sign * (0.1 + (0.2 - 0.1) * (abs(intensity)/5))	# min + (max spostamento nell'intervallo) * (intensita' normalizzata)
		print(intensity)
		#if force[0] < 0:
			# intensity = -intensity
			#intensity = 0
		if force[1] < 0:
			angularVelocity = -angularVelocity
		elif force[1] == 0:
			angularVelocity = 0
		# self.moveForward(intensity)
		# self.rotate(angularVelocity, degree)
		msg.linear.x = intensity
		msg.angular.z = angularVelocity
		self.velPub.publish(msg)
		# time.sleep(0.5)
		# self.velPub.publish(Twist())
		self.canRead = True

	def moveForward(self, intensity):
		direction = Twist()
		direction.linear.x = intensity
		self.velPub.publish(direction)
		time.sleep(0.5)
	

# ------------------------------------------------------------------------#


if __name__ == "__main__":
	try:
		robot = PotentialFieldsWander()
		#print(robot.sonarAngles)
		robot.start()
	except rospy.ROSInterruptException:
		print("ROSInterruptException")
	except KeyboardInterrupt:
		print("KeyboardInterrupt")
