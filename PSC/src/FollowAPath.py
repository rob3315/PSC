import rospy
from geometry_msgs.msg import Twist
from math import radians

class FollowAPath(self,path):
    def __init__(self):
        #initiliaze
        ropsy.init_node('FollowAPath', anonymous = False)
        
        # pour afficher le Twist() utilisé (la vitesse ?) queue_size = les chiffres après la virgule je crois
        # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
     
        # 10 HZ pour un garder une certaine fréquence dans la répétition d'une boucle
        r = rospy.Rate(10); 
        
        # variable temps pour bouger pendant une certaine durée :
        timer = 0

        # create two different Twist() variables.  One for moving forward.  One for turning 45 degrees.

        # let's go forward at 0.2 m/s
        move_cmd = Twist()
        move_cmd.linear.x = 0.1
    # by default angular.z is 0 so setting this isn't required
        
        #let's turn at 45 deg/s to the left ?
        turn_left_cmd = Twist()
        turn_left_cmd.linear.x = 0
        turn_left_cmd.angular.z = radians(45); # je ne sais pas comment distinguer lorsqu'on tourne à gauche et à droite. J'ai supposé que la vitesse angular.z permet de tourner dans le sens trigo donc vers la gauche

        #let's turn at 45 deg/s to the right ?
        turn_right_cmd = Twist()
        turn_right_cmd.linear.x = 0
        turn_right_cmd.angular.z = radians(315); #315 deg/s in radians/s. Peut-être que ça marche avec -45

        for i in range(len(path)):
            n,e,o=path[i] # on suppose que les données de l'algorithme de planification est un couple directions nors/est/ouest en m. Une seule des directions est non nulle. Le robot ne revient jamais sur ses pas 
            
            if n!=0 :  # on peut proabablement écrire les différents cas en moins de lignes mais j'ai préféré être prudente car je ne sais pas trop comment fonctionne rate.sleep()
                while timer < n*10 :
                    cmd_vel.publish(move_cmd)
                    rate.sleep()
                    timer=timer + 0.1 #fréquence de 10 Hz donc 0.1 secondes
                timer=0
            
            if e!=0 :
                while timer < 1 :
                    cmd_vel.publish(turn_right_cmd)
                    rate.sleep()
                    timer = timer + 0.1
                timer =0
                while timer < e*10 :
                    cmd_vel.publish(move_cmd)
                    rate.sleep()
                    timer=timer + 0.1
                timer=0
            
                    
            if o!=0 :
                while timer < 1 :
                    cmd_vel.publish(turn_left_cmd)
                    rate.sleep()
                    timer = timer + 0.1
                timer =0
                while timer < o*10 :
                    cmd_vel.publish(move_cmd)
                    rate.sleep()
                    timer=timer + 0.1
                timer=0
 
if __name__ == '__main__':
    try:
        path.FollowAPath()
    except:
        rospy.loginfo("node terminated.")

