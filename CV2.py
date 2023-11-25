import cv2
import numpy as np

def umbralizar_imagen(imagen, umbral):
    _, imagen_umbralizada = cv2.threshold(imagen, umbral, 255, cv2.THRESH_BINARY)
    return imagen_umbralizada

def ajustar_imagenes(imagen1, imagen2):
    # Reescalar imágenes
    imagen1_reescalada = reescalar_imagen(imagen1, escala)
    imagen2_reescalada = reescalar_imagen(imagen2, escala)

    # Aplicar traslación para matching 
    filas, columnas = imagen1_reescalada.shape[:2]
    M = np.float32([[1, 0, traslacion], [0, 1, 0]])
    imagen1_ajustada = cv2.warpAffine(imagen1_reescalada, M, (columnas, filas))
    imagen2_ajustada = cv2.warpAffine(imagen2_reescalada, M, (columnas, filas))


    return imagen1, imagen2

def calcular_iou(imagen1, imagen2):
    # Umbralizar imágenes
    umbral = 128
    imagen1_binaria = umbralizar_imagen(imagen1, umbral)
    imagen2_binaria = umbralizar_imagen(imagen2, umbral)

    # Ajustar imágenes
    imagen1_ajustada, imagen2_ajustada = ajustar_imagenes(imagen1_binaria, imagen2_binaria)

    # Calcular intersección
    interseccion = cv2.bitwise_and(imagen1_ajustada, imagen2_ajustada)

    # Calcular unión
    union = cv2.bitwise_or(imagen1_ajustada, imagen2_ajustada)

    # Calcular IoU
    iou = np.sum(interseccion) / np.sum(union)

    return iou

imagen1 = cv2.imread('trayT.png', cv2.IMREAD_GRAYSCALE)
imagen2 = cv2.imread('trayReal.png', cv2.IMREAD_GRAYSCALE)


iou_resultado = calcular_iou(imagen1, imagen2)
print(f"IoU: {iou_resultado}")
