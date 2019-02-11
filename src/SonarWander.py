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



class SonarWander:
	def __init__(self):
		rospy.init_node("sonarrr", anonymous=True)
		self.velPub = rospy.Publisher("/RosAria/cmd_vel", Twist, queue_size = 1)
		self.sonarSub = rospy.Subscriber("/RosAria/sonar", PointCloud, self.readSonar)
		self.poseSub = rospy.Subscriber("/RosAria/pose", Odometry, self.updatePose)
		self.sonarReading = PointCloud()
		self.RAD = pi/180
		self.releaser = False
		self.pose = Odometry()
		self.distance = [i for i in range(0, 16)]		

		self.iter = 0
		self.readIter = maxint
		self.canRead = True

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
			print("sto leggendo...")
			for p in self.sonarReading.points:
				if 0 < i <= 6:
					print("sto leggendo i sensori da 1 a 6.")
					self.distance[i] = math.sqrt(p.x * p.x + p.y * p.y)

					if self.distance[i] <= 5:
						# consideriamo le letture fino a 5mt
						if self.distance[i] < 0.5:
							# attiva releaser di avoid
							self.releaser = True
							print("Lettura Sonar che ha fatto scattare il releaser:", i)
							# disabilito la lettura per tre secondi
							#print("indice ostacolo: ", i)
							#self.obstacleIndex = i # indice direzione primo ostacolo
				i = i + 1

	def updatePose(self, msg):
		self.pose = msg

	def wander(self):
		# spostamento casuale (a campi di potenziale)
		if self.canRead:
			angle = random.uniform(-90 * self.RAD, 90 * self.RAD)
		else:
			angle = 0
		return 0

	def avoid(self):
		# sussume wander
		#self.canRead = False
		sonarIndex = 0
		minValue = self.distance.index(min(self.distance))
		threshold = 3	# da provare
		index = minValue - 1;
		angle = 0
		# la direzione ce l'ho in self.obstacleIndex

		print(self.distance[minValue])
		
		while index != minValue - len(self.distance):
			if self.distance[index] >= threshold:
				angle = self.sonarAngles[index]
				break
			index  = index - 1
		### se nessuno supera la threshold, vado in direzione di massima distanza
		if angle == 0:
			angle = max(i for i in self.distance)
			index = self.distance.index(angle)
		print("Vado in direzione del sonar",index,":",self.distance[index])
		return angle

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
					print("Releaser acceso!")
					rotation = self.avoid()
					self.releaser = False
				
				self.move(rotation)
			time.sleep(1)

	def move(self, rotation):
		#print("Angolo: ", rotation)
		self.canRead = False
		direction = Twist()
		if rotation != 0:
			self.rotate(0.3, rotation)
		else:
			self.moveForward()
		self.canRead = True

	def moveForward(self):
		direction = Twist()
		direction.linear.x = 0.5
		self.velPub.publish(direction)
		time.sleep(1)
	

#------------------------------------------------------------------------#	


if __name__ == "__main__":
	try:
		robot = SonarWander()
		#print(robot.sonarAngles)
		robot.start()
	except rospy.ROSInterruptException:
		print "ROSInterruptException"
	except KeyboardInterrupt:
		print "KeyboardInterrupt"