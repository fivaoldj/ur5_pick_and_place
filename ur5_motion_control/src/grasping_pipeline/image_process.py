import sys
import rospy
import numpy as np
import torch
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

class ImageProcessor:
    def __init__(self):
        self.device = torch.device("cuda")  # Используйте GPU

        torch.nn.Module.dump_patches = True
        self.model = torch.load('../ggcnn/ggcnn_weights_cornell/ggcnn_epoch_23_cornell')
        rospy.init_node('ggcnn_grasp_detector')
        rospy.Subscriber('/camera_3_stereo/points2', PointCloud2, self.pointcloud_callback)
         
        # Переменные для хранения данных
        self.depth_image = None
        self.grasp_params = None  # Здесь будут сохраняться параметры захвата

    def pointcloud_callback(self, data):
        """
        Callback для получения данных из PointCloud2 и преобразования их в глубинную карту.
        """
        try:
            # Преобразование PointCloud2 в numpy-массив
            points = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
            point_array = np.array(list(points))

            # Проверяем, что облако точек не пустое
            if point_array.shape[0] == 0:
                rospy.logwarn("PointCloud2 пустое!")
                return
            
            # Преобразование в глубинную карту
            depth_image = self.pointcloud_to_depth_map(point_array)

            # Передача глубинной карты в нейросеть
            self.grasp_params = self.process_with_ggcnn(depth_image)

            # Вывод результатов
            rospy.loginfo(f"Grasp Params: {self.grasp_params}")
        except Exception as e:
            rospy.logerr(f"Ошибка при обработке PointCloud2: {e}")

    def pointcloud_to_depth_map(self, point_array):
        """
        Преобразует облако точек в глубинную карту.
        """
        # Настройка параметров камеры
        IMAGE_WIDTH = 640
        IMAGE_HEIGHT = 480
        FX = 525.0  # Фокусное расстояние по X
        FY = 525.0  # Фокусное расстояние по Y
        CX = IMAGE_WIDTH / 2
        CY = IMAGE_HEIGHT / 2

        # Глубинная карта
        depth_image = np.full((IMAGE_HEIGHT, IMAGE_WIDTH), np.nan, dtype=np.float32)

        for point in point_array:
            x, y, z = point
            if z > 0:  # Отбрасываем точки за камерой
                u = int((x * FX / z) + CX)
                v = int((y * FY / z) + CY)
                if 0 <= u < IMAGE_WIDTH and 0 <= v < IMAGE_HEIGHT:
                    depth_image[v, u] = z

        # Заполняем пробелы методом интерполяции
        depth_image = self.fill_nan_gaps(depth_image)

        return depth_image

    def fill_nan_gaps(self, depth_image):
        """
        Заполняет пробелы (NaN) в глубинной карте методом интерполяции.
        """
        mask = np.isnan(depth_image)
        depth_image[mask] = 0
        depth_image = cv2.inpaint(depth_image.astype(np.float32), mask.astype(np.uint8), inpaintRadius=3, flags=cv2.INPAINT_TELEA)
        return depth_image

    def process_with_ggcnn(self, depth_image):
        """
        Передает глубинную карту в нейросеть GGCNN и возвращает параметры захвата.
        """
        # Преобразование глубинной карты в формат тензора
        depth_image = (depth_image - np.nanmin(depth_image)) / (np.nanmax(depth_image) - np.nanmin(depth_image))
        depth_tensor = torch.tensor(depth_image, dtype=torch.float32).unsqueeze(0).unsqueeze(0).to(self.device)

        # Прогон через модель
        with torch.no_grad():
            output = self.model(depth_tensor)

        # Извлечение параметров захвата
        q_img, angle_img, width_img = output

        # Преобразование тензоров в numpy
        q_img = q_img.cpu().squeeze().numpy()
        angle_img = angle_img.cpu().squeeze().numpy()
        width_img = width_img.cpu().squeeze().numpy()

        # Поиск лучшей точки захвата
        max_q_idx = np.unravel_index(np.argmax(q_img), q_img.shape)
        grasp_x, grasp_y = max_q_idx[1], max_q_idx[0]
        grasp_theta = angle_img[max_q_idx]
        grasp_width = width_img[max_q_idx]

        return {
            "x": grasp_x,
            "y": grasp_y,
            "z": depth_image[grasp_y, grasp_x],
            "theta": grasp_theta,
            "width": grasp_width,
        }


class GGCNNGraspDetector: