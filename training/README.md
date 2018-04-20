# Training the Traffic Light Classifier

## 1. Installation
Install the Tensorflow Object Detection API per the [install Instructions](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md). 

_Note: SHA 4a705e08ca8f28bd05bdc2287211ad8aba0bf98a was checked out to use an earier version of the API._

The following folder structure is used for this project:
```
+data
   -tf_record script
   -label_map file
   -train TFRecord file* 
   -eval TFRecord file*
   # *Location dependent on space constraints
+models
   +faster_rcnn_resnet101_lowproposals
      -pipeline config file
      +train
      +eval
      +fine_tuned_model
   +rfcn_resnet101
      -pipeline config file
      +train
      +eval
      +fine_tuned_model
```

## 1. Prepare the Dataset

A python script and label YAML files are used to create the [TFRecord file format](https://www.tensorflow.org/api_guides/python/python_io#tfrecords_format_details) per the Tensorflow Object Detection API.  The [Bosch Small Traffic Lights](https://hci.iwr.uni-heidelberg.de/node/6132) and the [Kaggle LISA Traffic Light](https://www.kaggle.com/mbornoe/lisa-traffic-light-dataset/version/2) datasets are used.   

1. Download the train_rgb and test_rgb zip files from the [Bosch Small Traffic Lights](https://hci.iwr.uni-heidelberg.de/node/6132) and the [Kaggle LISA Traffic Light](https://www.kaggle.com/mbornoe/lisa-traffic-light-dataset/version/2) dataset (only dayTrain was used).  
2. Edit the paths used in [tf_record script](data/tf_record.py) as needed. 
3. `python tf_record.py --output_dir ${PATH_TO_DATA_DIR}`
4. Use tl_train.record to train your model and tl_val.record to evaluate a trained model.

_Reference [Using Your Own Dataset](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/using_your_own_dataset.md)_

## 2. Configure the Job
Within the pretrained models folders are configuration files.  Update these files as needed to modify the training and evaluation parameters.  The **input_path** parameters should be set to the location of your  tl_train.record and tl_val.record files.
Faster RCNN Example: [pipeline.config](models/faster_rcnn_resnet101_lowproposals/pipeline.config)

_Reference [Configuring the Object Detection Training Pipeline](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/configuring_jobs.md)_

## 3. Training Locally
A local training job can be run with the following command:

```bash
# From the training directory
python train.py \
    --logtostderr \
    --pipeline_config_path=models/${DESIRED_MODEL}/pipeline.conf \
    --train_dir=models/${DESIRED_MODEL}/train
```
By default, the training job will run indefinitely until the user kills it.

While training, use Tensorboard to monitor the progress:
```bash
tensorboard --logdir=models/${DESIRED_MODEL}/train
```

Evaluation is run as a separate job. The eval job will periodically poll the
train directory for new checkpoints and evaluate them on a test dataset. The job can be run using the following command:

```bash
python eval.py \
    --logtostderr \
    --pipeline_config_path=models/${DESIRED_MODEL}/pipeline.conf \
    --checkpoint_dir=models/${DESIRED_MODEL}/train \
    --eval_dir=models/${DESIRED_MODEL}/eval
```

_Reference [Running Locally](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/running_locally.md)_

## Export the Trained Model
After your model has been trained, you should export it to a Tensorflow
graph proto. A checkpoint will typically consist of three files:

* model.ckpt-${CHECKPOINT_NUMBER}.data-00000-of-00001,
* model.ckpt-${CHECKPOINT_NUMBER}.index
* model.ckpt-${CHECKPOINT_NUMBER}.meta

After you've identified a candidate checkpoint to export, run the following
command from tensorflow/models/research:

``` bash
# From tensorflow/models/research/
python object_detection/export_inference_graph.py \
    --input_type image_tensor \
    --pipeline_config_path models/${DESIRED_MODEL}/pipeline.conf \
    --trained_checkpoint_prefix models/${DESIRED_MODEL}/train/model.chpt-${DESIRED_MODEL} \
    --output_directory models/${DESIRED_MODEL}/fine_tuned_model
```
The `frozen_inference_graph.pb` is now trained for use.

_Reference [Exporting a trained model for inference](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/exporting_models.md)_


