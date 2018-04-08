# Preparing The TFRecord for Training

These files create the [TFRecord file format](https://www.tensorflow.org/api_guides/python/python_io#tfrecords_format_details) per the Tensorflow Object Detection API.  The [Bosch Small Traffic Lights](https://hci.iwr.uni-heidelberg.de/node/6132) and the [Kaggle LISA Traffic Light](https://www.kaggle.com/mbornoe/lisa-traffic-light-dataset/version/2) datasets are used.   

### Instructions for Use

1. Install the API per the [install Instructions](models/research/object_detection/g3doc/installation.md)   
2. Edit the paths used in [tf record script](tf_record.py) as needed.
3. Run `python tf_record.py --output_path training.record`
4. Use training.record to train your model

Note: Please refer to [Using Your Own Dataset](models/research/object_detection/g3doc/using_your_own_dataset.md) for more details.