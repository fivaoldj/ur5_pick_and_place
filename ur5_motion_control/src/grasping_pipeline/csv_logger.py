import csv
import os

class CSVLogger:
    def __init__(self, file_path):
        """
        Инициализация логгера.
        :param file_path: Путь к CSV файлу, где будут сохраняться данные.
        """
        self.file_path = file_path
        
        # Проверяем, существует ли файл. Если нет, создаем его с заголовками.
        if not os.path.exists(self.file_path):
            with open(self.file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["trial_number", "success", "descriptor"])

    def log(self, success, descriptor):
        """
        Логирование данных в CSV файл.
        :param success: Успех или не успех (1 или 0).
        :param descriptor: Строковое описание (descriptor).
        """
        # Определяем номер нового испытания.
        trial_number = 0
        if os.path.exists(self.file_path):
            with open(self.file_path, mode='r') as file:
                reader = csv.reader(file)
                rows = list(reader)
                if len(rows) > 1:  # Если есть данные (помимо заголовка)
                    last_row = rows[-1]
                    trial_number = int(last_row[0]) + 1

        # Записываем новую строку в CSV файл.
        with open(self.file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([trial_number, success, descriptor])

    def read_logs(self):
        """
        Читает и возвращает все записи из CSV файла.
        :return: Список записей из CSV файла.
        """
        if os.path.exists(self.file_path):
            with open(self.file_path, mode='r') as file:
                reader = csv.DictReader(file)
                return list(reader)
        return []