import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import uic
import rospy
from std_msgs.msg import Float32MultiArray, String

class MiNodoROS(QMainWindow):
    def __init__(self):
        super(MiNodoROS, self).__init__()
        uic.loadUi('interfaz.ui', self)  # Carga el diseño de la interfaz
        self.initUI()
        
        rospy.init_node('mi_nodo_pyqt', anonymous=True)
        self.publisher = rospy.Publisher('cDirecta', Float32MultiArray, queue_size=1)
        rospy.Subscriber('resultado', String, self.callback)
        
    def initUI(self):
        self.boton_publicar.clicked.connect(self.publicar_datos)
        
    def publicar_datos(self):
        texto = self.lineEdit.text()
        try:
            valores = [float(i) for i in texto.split()]
            mensaje = Float32MultiArray(data=valores)
            self.publisher.publish(mensaje)
        except ValueError:
            rospy.loginfo("Por favor, introduce números separados por espacios.")
        
    def callback(self, data):
        # mofidicar en base al nombre del QLineEdit donde se desee mostrar el dato
        # tener en cuenta la posición del flotante en el arreglo 
        dato = data.data[0] + 1
        rospy.loginfo(rospy.get_caller_id() + " Mensaje recibido: %s", data.data)
        self.lineEdit.setText(str(data.data[0]))

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ventana = MiNodoROS()
    ventana.show()
    sys.exit(app.exec_())
