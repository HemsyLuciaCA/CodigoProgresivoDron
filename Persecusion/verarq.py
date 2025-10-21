import torch
from ultralytics import YOLO
from models.common import DetectMultiBackend  # Asegúrate de tener el repositorio de YOLOv8 clonado

model = DetectMultiBackend('YoloV8n.pt')
print(model)  # Esto imprimirá la arquitectura del modelo

from torchviz import make_dot

x = torch.randn(1, 3, 640, 640)  # Cambia el tamaño según tu configuración
y = model(x)
dot = make_dot(y, params=dict(model.named_parameters()))
dot.render('model_architecture', format='png')
