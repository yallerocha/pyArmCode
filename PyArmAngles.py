import math

def main():
  pyArm = PyArmAngles()
  angles = pyArm.getAngles(6, 6, 6)
    
  print(f"Angulo Base: {angles[0]}")
  print(f"Angulo Junta 1: {angles[1]}")
  print(f"Angulo Junta 2: {angles[2]}")
  print(f"Angulo Junta 3: {angles[3]}")

class PyArmAngles:

  L1 = 16.0
  L2 = 16.0
  L3 = 14.0
  fi = 180.0

  '''Funcao que retorna os angulos necessarios 
  para alcancar uma determinada posicao com o pyArm'''
  def getAngles(self, x, y, z):

    joint3_x, joint3_y = self.joint3Coords(x, y, z)

    if(self.positionValidation(joint3_x, joint3_y)):

      angleB = self.angleBase(x, y)
      angle2 = self.angleJoint2(joint3_x, joint3_y)
      angle1 = self.angleJoint1(joint3_x, joint3_y, angle2)
      angle3 = self.angleJoint3(angle1, angle2)

      return angleB, angle1, angle2, angle3
    
    else:
      raise positionError("Position outside the workspace!")

  # Funcao que calcula o angulo da base
  def angleBase(self, x, y):
    rad = math.atan2(x, y)
    angle = 90.0 - math.degrees(rad)

    return angle
  
  # Funcao que calcula o angulo da junta 1
  def angleJoint1(self, x, y, angle2):
    if angle2 < 0:
      rad = math.atan2(x, y) + math.acos((x**2 + y**2 + self.L1**2 - self.L2**2) / (2 * self.L1 * math.sqrt(x**2 + y**2)))
    else:
      rad = math.atan2(x, y) - math.acos((x**2 + y**2 + self.L1**2 - self.L2**2) / (2 * self.L1 * math.sqrt(x**2 + y**2)))

    angle1 = math.degrees(rad)

    return angle1
  
  # Funcao que calcula o angulo da junta 2
  def angleJoint2(self, x, y):
    rad = math.acos((x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2))
    angle2 = math.degrees(rad)

    return angle2
  
  # Funcao que calcula o angulo da junta 3
  def angleJoint3(self, angle1, angle2):
    angle3 = self.fi - (angle1 + angle2)

    return angle3
  
  # Funcao que verifica se é possivel alcancar a posicao recebida
  def positionValidation(self, x, y):
    dist = math.sqrt(x**2 + y**2)

    if(dist <= self.L1 + self.L2):
      return True
    else:
      return False
    
  # Funcao que retorna o X e Y em relação a junta 3 para calcular os angulos
  def joint3Coords(self, x, y, z):
    joint3_x = math.hypot(x, y) - self.L3 * math.sin(self.fi)
    joint3_y = z - self.L3 * math.cos(self.fi)

    return joint3_x, joint3_y
  
class positionError(Exception):
  pass

if __name__ == "__main__":
    main()
