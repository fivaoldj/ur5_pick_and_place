# ur5_pick_and_place

## Описание проекта (project description)

В данном проекте будет решаться задача **pick&place** на базе коллаборативного робота-манипулятора **UR5**.

Данный проект будет разделён на 2 части:

- **Управление движением робота**. Реализация преимущественно на `С++` с использованием `ROS Noetic` и пакета `MoveIt`
- Получение и **обработка изображения** с камеры для последующего построения карты окружения с помощью метода **point cloud**

### Описание части, касающейся управления движением робота.
Предполагается разработать класс, который будет с помощью пакета MoveIt подключаться к роботу и управлять им по запросу клиента. Также предполагается, что у нас будет реализована серверная часть сервиса, который будет осуществлять перемещение робота в назначенную с помощью сервиса точку.

Вероятно, имеет смысл реализовать сервисы для установки некоторых настроек для `MoveIt`

## Сборка и запуск
Запуск и сборка будет проходить следующим образом. Сначала собирается каждая состовляющая проекта и потом запускается. Предполагается, что сборка будет реализована в виде скрипта `build.sh`, а запуск будет реализован в виде скрипта вроде `run.sh` 

### Планирование движением 

### Обработка изображения

## Архитектура проекта