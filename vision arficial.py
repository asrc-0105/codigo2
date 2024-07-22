import cv2
import numpy as np
import datetime
import logging
import os
import argparse
import tkinter as tk
from tkinter import Label
from PIL import Image, ImageTk

# Configuración del logging para registrar errores y eventos del sistema
logging.basicConfig(
    filename='system.log',
    level=logging.DEBUG,  # Se puede cambiar a INFO para menos detalles
    format='%(asctime)s - %(levelname)s - %(message)s'
)

def handle_exception(func):
    """
    Decorador para manejar excepciones y registrar errores.
    Utiliza el decorador para envolver funciones y manejar cualquier excepción que pueda ocurrir.
    """
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            logging.error(f"Error en {func.__name__}: {e}")
            raise
    return wrapper

@handle_exception
def load_classes(filename='coco.names'):
    """
    Carga las etiquetas de las clases desde un archivo.
    Args:
        filename (str): Nombre del archivo que contiene las etiquetas de las clases.
    Returns:
        list: Lista de nombres de clases.
    """
    if not os.path.exists(filename):
        raise FileNotFoundError(f"El archivo {filename} no existe.")
    with open(filename, 'r') as f:
        return [line.strip() for line in f.readlines()]

@handle_exception
def load_yolo_model(weights='yolov3.weights', config='yolov3.cfg'):
    """
    Carga el modelo YOLO y sus capas de salida.
    Args:
        weights (str): Ruta del archivo de pesos del modelo YOLO.
        config (str): Ruta del archivo de configuración del modelo YOLO.
    Returns:
        tuple: Modelo YOLO cargado y nombres de las capas de salida.
    """
    if not os.path.exists(weights):
        raise FileNotFoundError(f"El archivo {weights} no existe.")
    if not os.path.exists(config):
        raise FileNotFoundError(f"El archivo {config} no existe.")
    net = cv2.dnn.readNet(weights, config)
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    return net, output_layers

@handle_exception
def prepare_image(image, new_size=(416, 416)):
    """
    Prepara la imagen para el modelo YOLO: redimensiona y normaliza.
    Args:
        image (numpy.ndarray): Imagen a procesar.
        new_size (tuple): Nuevo tamaño de la imagen (ancho, alto).
    Returns:
        tuple: Blob creado a partir de la imagen y dimensiones de la imagen redimensionada.
    """
    resized_image = cv2.resize(image, new_size)
    height, width, _ = resized_image.shape
    blob = cv2.dnn.blobFromImage(resized_image, scalefactor=0.00392, size=new_size, swapRB=True, crop=False)
    return blob, height, width

@handle_exception
def post_process(net, output_layers, blob, height, width, image, classes, confidence_threshold=0.5, nms_threshold=0.4):
    """
    Procesa la imagen con el modelo YOLO para detección de objetos.
    Args:
        net (cv2.dnn_Net): Modelo YOLO cargado.
        output_layers (list): Nombres de las capas de salida del modelo.
        blob (numpy.ndarray): Blob creado a partir de la imagen.
        height (int): Altura de la imagen original.
        width (int): Ancho de la imagen original.
        image (numpy.ndarray): Imagen original.
        classes (list): Lista de nombres de clases.
        confidence_threshold (float): Umbral de confianza para filtrar detecciones.
        nms_threshold (float): Umbral para la supresión de no máximos.
    Returns:
        numpy.ndarray: Imagen con las detecciones dibujadas.
    """
    net.setInput(blob)
    outs = net.forward(output_layers)

    class_ids = []
    confidences = []
    boxes = []

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > confidence_threshold:
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, confidence_threshold, nms_threshold)

    output_image = np.copy(image)
    colors = np.random.uniform(0, 255, size=(len(classes), 3))

    for i in indexes.flatten():
        x, y, w, h = boxes[i]
        label = str(classes[class_ids[i]])
        color = colors[class_ids[i]]
        cv2.rectangle(output_image, (x, y), (x + w, y + h), color, 2)
        cv2.putText(output_image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    return output_image

@handle_exception
def display_image_tkinter(image, window_name='Detección de Objetos'):
    """
    Muestra la imagen con detecciones en una ventana emergente usando Tkinter.
    Args:
        image (numpy.ndarray): Imagen a mostrar.
        window_name (str): Nombre de la ventana.
    """
    root = tk.Tk()
    root.title(window_name)

    image_pil = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    image_tk = ImageTk.PhotoImage(image_pil)

    label = Label(root, image=image_tk)
    label.pack()

    root.mainloop()

@handle_exception
def save_image(image, filename='output.jpg'):
    """
    Guarda la imagen procesada en un archivo.
    Args:
        image (numpy.ndarray): Imagen a guardar.
        filename (str): Nombre del archivo de salida.
    """
    cv2.imwrite(filename, image)
    logging.info(f"Imagen guardada como {filename}")

@handle_exception
def process_and_display_image(image_path, output_path=None, confidence_threshold=0.5, nms_threshold=0.4):
    """
    Carga una imagen desde el archivo, procesa la imagen con el modelo YOLO y muestra las detecciones en una ventana Tkinter.
    Args:
        image_path (str): Ruta de la imagen a procesar.
        output_path (str): Ruta para guardar la imagen procesada (opcional).
        confidence_threshold (float): Umbral de confianza para filtrar detecciones.
        nms_threshold (float): Umbral para la supresión de no máximos.
    """
    if not os.path.exists(image_path):
        raise FileNotFoundError(f"El archivo de imagen {image_path} no existe.")
    
    image = cv2.imread(image_path)
    if image is None:
        raise ValueError(f"No se pudo cargar la imagen {image_path}. Verifique el formato del archivo.")

    blob, height, width = prepare_image(image)
    output_image = post_process(net, output_layers, blob, height, width, image, classes, confidence_threshold, nms_threshold)
    display_image_tkinter(output_image)
    if output_path:
        save_image(output_image, output_path)

@handle_exception
def process_video(video_path, output_path=None, confidence_threshold=0.5, nms_threshold=0.4):
    """
    Procesa un video con el modelo YOLO, mostrando y/o guardando los cuadros procesados.
    Args:
        video_path (str): Ruta del video a procesar.
        output_path (str): Ruta para guardar el video procesado (opcional).
        confidence_threshold (float): Umbral de confianza para filtrar detecciones.
        nms_threshold (float): Umbral para la supresión de no máximos.
    """
    if not os.path.exists(video_path):
        raise FileNotFoundError(f"El archivo de video {video_path} no existe.")
    
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise ValueError(f"No se pudo abrir el video {video_path}. Verifique el formato del archivo.")
    
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = None
    if output_path:
        out = cv2.VideoWriter(output_path, fourcc, 20.0, (int(cap.get(3)), int(cap.get(4))))
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        blob, height, width = prepare_image(frame)
        output_frame = post_process(net, output_layers, blob, height, width, frame, classes, confidence_threshold, nms_threshold)
        cv2.imshow('Detección de Objetos', output_frame)
        if output_path:
            out.write(output_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    if output_path:
        out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Script de detección de objetos usando YOLO.")
    parser.add_argument('--image', type=str, help="Ruta de la imagen a procesar.")
    parser.add_argument('--video', type=str, help="Ruta del video a procesar.")
    parser.add_argument('--output', type=str, help="Ruta para guardar la salida procesada.")
    parser.add_argument('--weights', type=str, default='yolov3.weights', help="Ruta del archivo de pesos de YOLO.")
    parser.add_argument('--config', type=str, default='yolov3.cfg', help="Ruta del archivo de configuración de YOLO.")
    parser.add_argument('--classes', type=str, default='coco.names', help="Ruta del archivo de etiquetas de clases.")
    parser.add_argument('--conf-thresh', type=float, default=0.5, help="Umbral de confianza para detecciones.")
    parser.add_argument('--nms-thresh', type=float, default=0.4, help="Umbral para la supresión de no máximos.")
    parser.add_argument('--no-display', action='store_true', help="Desactiva la visualización en ventana Tkinter.")

    args = parser.parse_args()

    # Validación de los argumentos
    if not (args.image or args.video):
        parser.error("Debes proporcionar una ruta para la imagen o el video a procesar.")

    classes = load_classes(args.classes)
    net, output_layers = load_yolo_model(args.weights, args.config)

    if args.image:
        process_and_display_image(args.image, args.output, args.conf_thresh, args.nms_thresh)
    elif args.video:
        process_video(args.video, args.output, args.conf_thresh, args.nms_thresh)
    else:
        print("Por favor, proporciona una ruta para la imagen o el video a procesar.")
