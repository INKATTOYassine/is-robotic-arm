import serial
import time

class RobotCom:
    def __init__(self, port, baudrate=115200):
        # Ouverture du port série
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2) # Attendre le reboot automatique de l'Arduino
        print("Connecté à l'Arduino sur", port)

    def send_trajectory(self, j1, j2, j3, j4, j5):
        """
        Envoie une consigne de position formatée avec les marqueurs < >
        Les valeurs en entrée doivent être les STEPS calculés par votre simulateur IK.
        """
        # Formatage strict de la trame
        frame = f"<T,{int(j1)},{int(j2)},{int(j3)},{int(j4)},{int(j5)}>"
        
        # Envoi en encodage binaire ASCII
        self.ser.write(frame.encode('ascii'))
        print(f"Envoyé : {frame}")
        
        # Attente de l'accusé de réception (ACK)
        response = self.wait_for_response()
        return response

    def wait_for_response(self):
        start_time = time.time()
        while (time.time() - start_time) < 2.0: # Timeout de sécurité de 2 secondes
            if self.ser.in_waiting > 0:
                resp = self.ser.readline().decode('ascii').strip()
                print(f"Reçu   : {resp}")
                return resp
        print("Erreur : Délai d'attente dépassé (Timeout)")
        return "TIMEOUT"

# --- TEST ---
if __name__ == "__main__":
    # Remplacez 'COM3' par '/dev/ttyUSB0' ou '/dev/ttyACM0' sur Raspberry Pi / Linux
    robot = RobotCom(port='COM3') 
    
    # Envoi de la consigne (exemple : faire tourner l'épaule et le coude)
    robot.send_trajectory(0, 2000, -1000, 0, 0)