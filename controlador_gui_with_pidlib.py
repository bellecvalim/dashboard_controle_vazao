import sys
import time
from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import QVBoxLayout, QGridLayout, QApplication, QLabel, QLineEdit, QPushButton, QGroupBox, QHBoxLayout, QMessageBox
import numpy as np
import pyqtgraph as pg
from w1thermsensor import W1ThermSensor
import RPi.GPIO as GPIO
import procbridge
import time
from simple_pid import PID

OUT_temp = 0.0
motor = 80.0
    
GPIO.setwarnings(False)

class App(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(App, self).__init__(parent)

        # define o tamanho da tela
        self.setFixedSize(1200,800)

        ## cria janela principal
        self.mainbox = QtGui.QWidget()
        self.setCentralWidget(self.mainbox)

        ## cria um grid para os comandos
        self.grid = QGridLayout()
        self.grid.setSpacing(10)
        self.mainbox.setLayout(self.grid)

        # inicializa o painel de controle
        self._initialize_control_panel()
        
        # inicializa os paineis de gráficos
        self._initialize_charts()

        # inicializa os parâmetros para cálculo
        self._initialize_parameters()

        # variável para indicar se o controle está em execução
        self._started = False

        # inicializa os vetores dos graficos
        self.Y_var_controlada = []
        self.Y_var_manipulada = []
        self.Y_tanque_frio = []
        self.Y_var_setpoint = []
        self.X = []

        # inicializa as variáveis
        self.sp = None
        self.kc = None
        self.ki = None
        self.kd = None
        self.pid = None

        # inicializa o sensor
        self.sensor_var_controlada = W1ThermSensor.get_available_sensors()[0]
        self.sensor_tanque_frio = W1ThermSensor.get_available_sensors()[1]

        # inicializa o controlador do motor
        self._initialize_motor()
        
        

    ##########################################################
    # Inicializa os parâmetros erro, delta_t, integral e ise
    ##########################################################
    def _initialize_parameters(self):
        self.erro_anterior = None
        self.erro_atual = None
        self.delta_t = 1 # 1 segundo
        self.integral = 0
        self.ise = 0

    ##########################################################
    # Inicializa o painel de control
    ##########################################################
    def _initialize_control_panel(self):

        # campo para definir o setpoint
        self.spLabel = QLabel('SP')
        self.spEdit = QLineEdit()
        self.grid.addWidget(self.spLabel, 2, 0)
        self.grid.addWidget(self.spEdit, 2, 1)

        # campos para definir os parâmetros de controle
        self.parametrosLabel = QLabel('Parâmetros do controlador')
        self.parametrosLabel.setFixedHeight(30)
        self.kcLabel = QLabel('Kc')
        self.kcEdit = QLineEdit()
        self.kiLabel = QLabel('Ki')
        self.kiEdit = QLineEdit()
        self.kdLabel = QLabel('Kd')
        self.kdEdit = QLineEdit()

        self.grid.addWidget(self.parametrosLabel, 1, 2, 1, 3)
        self.grid.addWidget(self.kcLabel, 2, 2)
        self.grid.addWidget(self.kcEdit, 2, 3)
        self.grid.addWidget(self.kiLabel, 3, 2)
        self.grid.addWidget(self.kiEdit, 3, 3)
        self.grid.addWidget(self.kdLabel, 4, 2)
        self.grid.addWidget(self.kdEdit, 4, 3)

        # botões start-stop
        self.gridButtons = QHBoxLayout()
        self.groupBoxButtons = QGroupBox()
        self.groupBoxButtons.setLayout(self.gridButtons)
        self.grid.addWidget(self.groupBoxButtons, 5, 0, 1, 4)
        
        self.startButton = QPushButton('Start')
        self.startButton.clicked.connect(self.start)
        # self.grid.addWidget(self.startButton, 5, 0, 1, 2)
        self.gridButtons.addWidget(self.startButton)

        self.stopButton = QPushButton('Stop')
        self.stopButton.clicked.connect(self.stop)
        # self.grid.addWidget(self.stopButton, 5, 2, 1, 2)
        self.gridButtons.addWidget(self.stopButton)

        self.clearButton = QPushButton('Clear')
        self.clearButton.clicked.connect(self.clear)
        self.gridButtons.addWidget(self.clearButton)

        #self.ISELabel = QLabel('')
        #self.grid.addWidget(self.ISELabel, 8, 4)
    
    ##########################################################
    # Inicializa os gráficos 
    ##########################################################
    def _initialize_charts(self):

        # área para gráfico no canto superior direito
        self.canvas1 = pg.GraphicsLayoutWidget()
        self.canvas1.setBackground('w')
        self.canvas1.setFixedHeight(300)
        self.canvas1.setFixedWidth(350)
        self.grid.addWidget(self.canvas1, 1, 4, 6, 1)
        
        self.plot_tanque_frio = self.canvas1.addPlot(title='Temperatura do tanque frio (˚C)', labels={'left':'Temperatura (˚C)'})
        self.grafico_tanque_frio = self.plot_tanque_frio.plot(pen=pg.mkPen('r', width=2))

        ## Create canvas (where we will put charts)
        self.canvas = pg.GraphicsLayoutWidget()
        self.canvas.setBackground('w')
        self.grid.addWidget(self.canvas, 7, 0, 1, 5)

        self.canvas.nextColumn()

        # cria o gráfico de variavel manipulada
        self.plot_var_manipulada = self.canvas.addPlot(title='Variável manipulada', labels={'left': 'Ângulo do motor (Graus)'})
        self.grafico_var_manipulada = self.plot_var_manipulada.plot(pen=pg.mkPen('r', width=2))

        # cria uma coluna no canvas
        self.canvas.nextColumn()

        #  line plot
        self.plot_var_controlada = self.canvas.addPlot(title='Variável controlada', labels={'left': 'Temperatura do reator (°C)'})
        self.grafico_var_controlada = self.plot_var_controlada.plot(pen=pg.mkPen('r', width=2))
        self.grafico_var_target = self.plot_var_controlada.plot(pen=pg.mkPen('g', width=3))

    ##########################################################
    # Executa o controle e faz update periódico da tela 
    ##########################################################
    def _update(self):
        if not self._started:
            return

        t_tanque_frio = self.sensor_tanque_frio.get_temperature()
        t_var_controlada = self.sensor_var_controlada.get_temperature()
        now = int(round(time.time()*1000))
        #self.client.request("pub", {"TemperatureVariable":float(t-var-controlada), "TemperatureTank":float(t_tanque_frio), "Timestamp":now})
        angulo = self.calculate_angle(t_var_controlada)
        setpoint = float(self.spEdit.text())

        

        if len(self.X) == 0: 
            # primeira leitura
            self.X = [1]

            self.Y_tanque_frio = [t_tanque_frio]
            self.Y_var_controlada = [t_var_controlada]
            self.Y_var_manipulada = [angulo]
            self.Y_var_setpoint = [setpoint]
        else:
            # demais leituras
            self.X = np.append(self.X, [self.X[-1:][0] + 1])

            self.Y_tanque_frio = np.append(self.Y_tanque_frio, [t_tanque_frio])
            self.Y_var_controlada = np.append(self.Y_var_controlada, [t_var_controlada])
            self.Y_var_manipulada = np.append(self.Y_var_manipulada, [angulo])
            self.Y_var_setpoint = np.append(self.Y_var_setpoint, [setpoint])
            
        self._update_charts()

        self.set_angle(angulo)

        #self.ISELabel.setText('Reabastecer tanque frio com gelo!')

        QtCore.QTimer.singleShot(int(1000 * self.delta_t), self._update)
    
    ############################################
    # Atualiza as informações dos gráficos
    ############################################
    def _update_charts(self):
        self.grafico_tanque_frio.setData(self.X, self.Y_tanque_frio)
        self.grafico_var_controlada.setData(self.X, self.Y_var_controlada)
        self.grafico_var_manipulada.setData(self.X, self.Y_var_manipulada)
        self.grafico_var_target.setData(self.X, self.Y_var_setpoint)

    ########################################################## 
    # Bloqueia ou desbloqueia a edição dos campos
    ########################################################## 
    def _set_read_only(self, read_only):
        self.kcEdit.setReadOnly(read_only)
        self.kdEdit.setReadOnly(read_only)
        self.kiEdit.setReadOnly(read_only)
        self.spEdit.setReadOnly(read_only)


    ###########################################
    # Calcula o angulo do motor
    ###########################################

    def calculate_angle(self, t_var_controlada):
        global OUT_temp
        global motor
        self.erro_anterior = self.erro_atual

        #self.erro_atual = min((t_var_controlada - self.sp) / self.sp, 1)
        self.erro_atual = t_var_controlada - self.sp
        self.integral = self.integral + self.erro_atual*self.delta_t
        self.ise = self.ise + self.erro_atual**2*self.delta_t
        
    #     termo_p = self.erro_atual

    #     if self.ki > 0:
    #         termo_i = self.integral * self.delta_t
    #     else:
    #         termo_i = 0
        
    #     if self.kd > 0 and self.erro_anterior is not None:
    #         termo_d = (self.erro_atual - self.erro_anterior) / self.delta_t
    #     else:
    #         termo_d = 0
            
    #     OUT = self.kc * termo_p + self.ki*termo_i + self.kd * termo_d
             
    #    #if OUT < 0:
    #         #angulo = 0
    #     #else:
   
    #     #angulo_novo = max(int(OUT), 75)
    #     angulo_novo = 75+OUT
    #     angulo = min(angulo_novo, 170)
    #     print("%.2f \t %.2f \t%.2f \t %.2f \t %.2f \n"%(OUT, termo_p, termo_i, termo_d, motor))
    #     OUT_temp = OUT
    #     return angulo
        return self.pid(t_var_controlada)

    ############################################
    # Ações dos botões
    ############################################
    def start(self):
        try:
            self.sp = float(self.spEdit.text())
            self.kc = float(self.kcEdit.text())
            self.ki = float(self.kiEdit.text())
            self.kd = float(self.kdEdit.text())

            if len(self.Y_var_manipulada) > 0:
                self.set_angle(self.Y_var_manipulada[-1:][0])

            self._started = True
            self.pid = PID(Kp=self.kc, Ki=self.ki, Kd=self.kd, setpoint=self.sp, sample_time=1.0, output_limits=(75, 180))
            self._set_read_only(True)
            self._update()
        except Exception:
            QMessageBox.critical(self, "Erro", "Verifique os parâmetros")

    def stop(self):
        self._started = False
        self._set_read_only(False)
        self.set_angle(0)

    def clear(self):
        self.Y_tanque_frio = []
        self.Y_var_controlada = []
        self.Y_var_manipulada = []
        self.Y_var_setpoint = []
        self.X = []

        # self.ISELabel.setText('')

        self.set_angle(0)

        self._update_charts()

        self._initialize_parameters()


    ########################################################## 
    # PWM
    ########################################################## 
    def _initialize_motor(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(22, GPIO.OUT)
        self.pwm = GPIO.PWM(22, 50)

        self.pwm.start(0)
    
    def set_angle(self, angle):
        duty = angle / 18 + 2
        GPIO.output(22, True)
        self.pwm.ChangeDutyCycle(duty)

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    thisapp = App()
    thisapp.show()
    app.exec_()
    
    datafile = open('dados_tanque.txt','w')
    datafile.write("Tanque quente \t Tanque Frio")
    
    for i in range(len(thisapp.Y_tanque_frio)):
       datafile.write("%.2f \t %.2f \n" %(thisapp.Y_var_controlada[i], thisapp.Y_tanque_frio[i]))
       
    datafile.close()
    