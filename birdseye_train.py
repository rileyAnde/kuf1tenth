#Requirement Library
import os
os.environ["CUDA_VISIBLE_DEVICES"] = "-1" # If you want utilize GPU, uncomment this line
#from sklearn.utils import shuffle
import time
import subprocess
import numpy as np
import tensorflow as tf
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from tensorflow.keras.losses import huber
from tensorflow.keras.optimizers import Adam
import pickle
from birdseye import Birdseye

# Check GPU availability - You don't need a gpu to train this model
# os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
gpu_available = tf.test.is_gpu_available()
print('GPU AVAILABLE:', gpu_available)

#========================================================
# Functions
#========================================================
#Linear maping
def linear_map(x, x_min, x_max, y_min, y_max):
    """Linear mapping function."""
    return (x - x_min) / (x_max - x_min) * (y_max - y_min) + y_min

#Huber Loss
def huber_loss(y_true, y_pred, delta=1.0):
    error = np.abs(y_true - y_pred)
    loss = np.where(error <= delta, 0.5 * error**2, delta * (error - 0.5 * delta))
    mean_loss = np.mean(loss)
    return mean_loss
#========================================================
# Global Data
#========================================================

# Initialize lists for data
lidar = np.empty((0, 541))
servo = []
speed = []
test_lidar = np.empty((0, 541))
test_servo = []
test_speed = []
model_name = 'birdseye'
model_files = [
    './models/'+model_name+'_noquantized.tflite',
    './models/'+model_name+'_int8.tflite'
]
dataset_path = [
    #'./Dataset/out.bag', 
    #'./Dataset/f2.bag', 
    #'./Dataset/f4.bag',
    '/home/r478a194/f1tenth_omniverse/ros2_ws/joy_data4.pkl',
    '/home/r478a194/f1tenth_omniverse/ros2_ws/joy_data3.pkl'
]
loss_figure_path = './Figures/loss_curve.png'
down_sample_param = 2 # Down-sample Lidar data
lr = 5e-5
loss_function = 'huber'
batch_size = 64
num_epochs = 20
hz = 40

# Initialize variables for min and max speed
max_speed = 0
min_speed = 0

#========================================================
# Get Dataset
#========================================================

# Iterate through bag files
for pth in dataset_path:
    if not os.path.exists(pth):
        print(f"out.bag doesn't exist in {pth}")
        exit(0)
    #good_bag = rosbag.Bag(pth)

    lidar_data = []
    servo_data = []
    speed_data = []

    # Read messages from bag file
    # for topic, msg, t in good_bag.read_messages():
    #     if topic == 'Lidar':
    #         ranges = msg.ranges[::down_sample_param]
    #         lidar_data.append(ranges)
    #     if topic == 'Ackermann':
    #         data = msg.drive.steering_angle
    #         s_data = msg.drive.speed
            
    #         servo_data.append(data)
    #         if s_data > max_speed:
    #             max_speed = s_data
    #         speed_data.append(s_data)

    '''START RILEY LOADER'''
    with open(pth, "rb") as f:
        raw_data = pickle.load(f)
    
    transposed_data = []
    for i in range(len(raw_data[0])):
        lid, act = raw_data[0][i], raw_data[1][i]
        spee, serv = act
        lidar_data.append(np.array(lid))
        speed_data.append(spee)
        servo_data.append(serv)
    #print(lidar_data, speed_data, servo_data)
    #sampled_data = np.random.sample(transposed_data, 10000)

    '''END OF RILEY LOADER'''


    # Convert data to arrays
    lidar_data = np.array(lidar_data) 
    #print(lidar_data)
    servo_data = np.array(servo_data)
    print(servo_data)
    speed_data = np.array(speed_data)

    # # Shuffle data
    # concat_data = np.concatenate((servo_data[:, np.newaxis], speed_data[:, np.newaxis]), axis=1)
    # np.random.shuffle(concat_data)
    # shuffled_data = concat_data #np.random.shuffle(np.concatenate((servo_data[:, np.newaxis], speed_data[:, np.newaxis]), axis=1)) #, random_state=62)
    # np.random.shuffle(lidar_data)
    # shuffled_lidar_data = lidar_data

    combined = list(zip(lidar_data, servo_data, speed_data))
    np.random.shuffle(combined)

    # Unpack back into separate arrays
    shuffled_lidar_data, shuffled_servo_data, shuffled_speed_data = zip(*combined)
    shuffled_lidar_data = np.array(shuffled_lidar_data)
    shuffled_servo_data = np.array(shuffled_servo_data)
    shuffled_speed_data = np.array(shuffled_speed_data)

    shuffled_data = np.concatenate((shuffled_servo_data[:, np.newaxis], shuffled_speed_data[:, np.newaxis]), axis=1)

    # Split data into train and test sets
    train_ratio = 0.85
    train_samples = int(train_ratio * len(shuffled_lidar_data))
    x_train_bag, x_test_bag = shuffled_lidar_data[:train_samples], shuffled_lidar_data[train_samples:]

    # Extract servo and speed values
    y_train_bag = shuffled_data[:train_samples]
    y_test_bag = shuffled_data[train_samples:]

    # Extend lists with train and test data
    '''RILEY CHANGED TO CONCAT'''
    lidar = np.concatenate([lidar, x_train_bag])
    servo = np.concatenate([servo, y_train_bag[:, 0]])
    speed = np.concatenate([speed, y_train_bag[:, 1]])

    test_lidar = np.concatenate([test_lidar, x_test_bag])
    test_servo = np.concatenate([test_servo, y_test_bag[:, 0]])
    test_speed = np.concatenate([test_speed, y_test_bag[:, 1]])

    print(f'\nData in {pth}:')
    print(f'Shape of Train Data --- Lidar: {len(lidar)}, Servo: {len(servo)}, Speed: {len(speed)}')
    print(f'Shape of Test Data --- Lidar: {len(test_lidar)}, Servo: {len(test_servo)}, Speed: {len(test_speed)}')

# Calculate total number of samples
total_number_samples = len(lidar)

print(f'Overall Samples = {total_number_samples}')
lidar = np.asarray(lidar)
servo = np.asarray(servo)
speed = np.asarray(speed)
min_speed = np.min(speed)
max_speed = np.max(speed)
speed = linear_map(speed, min_speed, max_speed, 0, 1)
test_lidar = np.asarray(test_lidar)
test_servo = np.asarray(test_servo)
test_speed = np.asarray(test_speed)
test_speed = linear_map(test_speed, min_speed, max_speed, 0, 1)

print(f'Min_speed: {min_speed}')
print(f'Max_speed: {max_speed}')
print(f'Loaded {len(lidar)} Training samples ---- {(len(lidar)/total_number_samples)*100:0.2f}% of overall')
print(f'Loaded {len(test_lidar)} Testing samples ---- {(len(test_lidar)/total_number_samples)*100:0.2f}% of overall\n')

# Check array shapes
assert len(lidar) == len(servo) == len(speed)
assert len(test_lidar) == len(test_servo) == len(test_speed)

#======================================================
# Split Dataset
#======================================================

print('Splitting Data into Train/Test')
train_data = np.concatenate((servo[:, np.newaxis], speed[:, np.newaxis]), axis=1)
test_data =  np.concatenate((test_servo[:, np.newaxis], test_speed[:, np.newaxis]), axis=1)
# Check array shapes
print(f'Train Data(lidar): {lidar.shape}')
print(f'Train Data(servo, speed): {servo.shape}, {speed.shape}')
print(f'Test Data(lidar): {test_lidar.shape}')
print(f'Test Data(servo, speed): {test_servo.shape}, {test_speed.shape}')

#======================================================
# DNN Arch
#======================================================

print(f'num_lidar_range_values: {len(lidar[0])}')

model = tf.keras.Sequential([
    tf.keras.layers.Input(shape=(100, 100, 1)),
    tf.keras.layers.Conv2D(filters=24, kernel_size=(5, 5), strides=(2, 2), activation='relu'),
    tf.keras.layers.Conv2D(filters=36, kernel_size=(5, 5), strides=(2, 2), activation='relu'),
    tf.keras.layers.Conv2D(filters=48, kernel_size=(3, 3), strides=(2, 2), activation='relu'),
    tf.keras.layers.Conv2D(filters=64, kernel_size=(3, 3), activation='relu'),
    tf.keras.layers.Conv2D(filters=64, kernel_size=(3, 3), activation='relu'),
    tf.keras.layers.Flatten(),
    tf.keras.layers.Dense(100, activation='relu'),
    tf.keras.layers.Dense(50, activation='relu'),
    tf.keras.layers.Dense(10, activation='relu'),
    tf.keras.layers.Dense(2, activation='tanh')
])


#======================================================
# Model Compilation
#======================================================

optimizer = Adam(lr)
model.compile(optimizer=optimizer, loss=loss_function)
print(model.summary())

#======================================================
# Model Fit
#======================================================
start_time = time.time()
imgconverter = Birdseye(100)

# Convert lidar scans to 100x100x3 images
img_lidar = np.array([imgconverter.make(scan) for scan in lidar])
img_test_lidar = np.array([imgconverter.make(scan) for scan in test_lidar])

history = model.fit(img_lidar, np.concatenate((servo[:, np.newaxis], speed[:, np.newaxis]), axis=1),
                    epochs=num_epochs, batch_size=batch_size,
                    validation_data=(img_test_lidar, test_data))


print(f'=============>{int(time.time() - start_time)} seconds<=============')

# Plot training and validation losses
plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.title('Model Loss')
plt.ylabel('Loss')
plt.xlabel('Epoch')
plt.legend(['Train', 'Test'], loc='upper left')
os.makedirs(os.path.dirname(loss_figure_path), exist_ok=True)
plt.savefig(loss_figure_path)
plt.close()

#======================================================
# Model Evaluation
#======================================================

print("==========================================")
print("Model Evaluation")
print("==========================================")

# Evaluate test loss
test_loss = model.evaluate(img_test_lidar, test_data)
print(f'Overall Test Loss = {test_loss}')

# Calculate and print overall evaluation
y_pred = model.predict(img_test_lidar)
hl = huber_loss(test_data, y_pred)
print('\nOverall Evaluation:')
print(f'Overall Huber Loss: {hl:.3f}')

# Calculate and print speed evaluation
speed_y_pred = model.predict(img_test_lidar)[:, 1]
speed_test_loss = huber_loss(test_data[:, 1], speed_y_pred)
print("\nSpeed Evaluation:")
print(f"Speed Test Loss: {speed_test_loss}")

# Calculate and print servo evaluation
servo_y_pred = model.predict(img_test_lidar)[:, 0]
servo_test_loss = huber_loss(test_data[:, 0], servo_y_pred)
print("\nServo Evaluation:")
print(f"Servo Test Loss: {servo_test_loss}")

#======================================================
# Save Model
#======================================================
# Save non-quantized model
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()
tflite_model_path = './models/' + model_name + "_noquantized.tflite"
with open(tflite_model_path, 'wb') as f:
    f.write(tflite_model)
    print(f"{model_name}_noquantized.tflite is saved.")

# Save int8 quantized model
rep_32 = img_lidar.astype(np.float32)

def representative_data_gen():
    for input_value in rep_32:
        yield [np.expand_dims(input_value, axis=0)]


converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.representative_dataset = representative_data_gen
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
quantized_tflite_model = converter.convert()

tflite_model_path = './models/' + model_name + "_int8.tflite"
with open(tflite_model_path, 'wb') as f:
    f.write(quantized_tflite_model)
    print(f"{model_name}_int8.tflite is saved.")

print('Tf_lite Models also saved')

#======================================================
# Evaluated TfLite Model
#======================================================

def evaluate_model(model_path, test_lidar, test_data):
    """Evaluate TfLite model."""
    # Load the TFLite model
    interpreter = tf.lite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    input_index = interpreter.get_input_details()[0]["index"]
    output_details = interpreter.get_output_details()

    output_lidar = test_lidar
    output_servo = []
    output_speed = []

    period = 1.0 / hz

    # Initialize a list to store inference times in microseconds
    inference_times_micros = []

    # Iterate through the lidar data
    for lidar_data in output_lidar:
        # Preprocess lidar data for inference
        lidar_data = lidar_data.astype(np.float32)
        lidar_data = np.expand_dims(lidar_data, axis=0)

        # Check for empty lidar data
        if lidar_data is None:
            continue

        # Measure inference time
        ts = time.time()
        interpreter.set_tensor(input_index, lidar_data)
        interpreter.invoke()
        output = interpreter.get_tensor(output_details[0]['index'])
        dur = time.time() - ts

        # Convert inference time to microseconds
        inference_time_micros = dur * 1e6
        inference_times_micros.append(inference_time_micros)

        # Print inference time information
        if dur > period:
            print("%.3f: took %.2f microseconds - deadline miss." % (dur, int(dur * 1000000)))

        # Extract servo and speed output from the model
        servo = output[0, 0]
        speed = output[0, 1]

        # Append output servo and speed
        output_servo.append(servo)
        output_speed.append(speed)

    output_lidar = np.asarray(output_lidar)
    output_servo = np.asarray(output_servo)
    output_speed = np.asarray(output_speed)
    assert len(output_lidar) == len(output_servo) == len(output_speed)
    output = np.concatenate((output_servo[:, np.newaxis], output_speed[:, np.newaxis]), axis=1)
    y_pred = output

    # Calculate average and maximum inference times in microseconds
    arr = np.array(inference_times_micros)
    perc99 = np.percentile(arr, 99)
    arr = arr[arr < perc99]
    average_inference_time_micros = np.mean(arr)
    max_inference_time_micros = np.max(arr)

    # Print inference time statistics
    print("Model: ", model_path)
    print("Average Inference Time: %.2f microseconds" % average_inference_time_micros)
    print("Maximum Inference Time: %.2f microseconds" % max_inference_time_micros)

    return y_pred, inference_times_micros

# Initialize empty lists to store results for each model
all_inference_times_micros = []
for model_name in model_files:
    y_pred, inference_times_micros = evaluate_model(model_name, img_test_lidar, test_data)
    all_inference_times_micros.append(inference_times_micros)
    
    print(f'Huber Loss for {model_name}: {huber_loss(test_data, y_pred)}\n')

# Plot inference times
plt.figure()
for inference_times_micros in all_inference_times_micros:
    arr = np.array(inference_times_micros)
    perc99 = np.percentile(arr, 99)
    arr = arr[arr < perc99]
    plt.plot(arr)
plt.xlabel('Inference Iteration')
plt.ylabel('Inference Time (microseconds)')
plt.title('Inference Time per Iteration')
plt.legend(model_files)

print('End')