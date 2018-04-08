import tensorflow as tf
import sys
sys.path.insert(0, '/~/Documents/models/research/object_detection/utils')
import dataset_util

flags = tf.app.flags
flags.DEFINE_string('output_path', '', 'Path to output TFRecord')
FLAGS = flags.FLAGS

LABEL_DICT = {
    "Green": 1,
    "Red": 2,
    "GreenLeft": 3,
    "GreenRight": 4,
    "RedLeft": 5,
    "RedRight": 6,
    "Yellow": 7,
    "off": 8,
    "RedStraight": 9,
    "GreenStraight": 10,
    "GreenStraightLeft": 11,
    "GreenStraightRight": 12,
    "RedStraightLeft": 13,
    "RedStraightRight": 14
    }


def create_tf_example(example):
    if example['source'] == 'LISA'
        image_height = 960
        image_width = 1280
        image_format = 'jpeg'.encode()

    else
        # Bosch
        image_height = 720
        image_width = 1280
        image_format = 'png'.encode()
    
    filename = example['path']
    with tf.gfile.GFile(example['path'], 'rb') as fid:
        encoded_image = fid.read()
    
    xmins = []  # List of normalized left x coordinates in bounding box (1 per box)
    xmaxs = []  # List of normalized right x coordinates in bounding box (1 per box)
    ymins = []  # List of normalized top y coordinates in bounding box (1 per box)
    ymaxs = []  # List of normalized bottom y coordinates in bounding box (1 per box)
    classes_text = []  # List of string class name of bounding box (1 per box)
    classes = []  # List of integer class id of bounding box (1 per box)
    
    for box in example['boxes']:
        xmins.append(float(box['x_min'] / image_width))
        xmaxs.append(float(box['x_max'] / image_width))
        ymins.append(float(box['y_min'] / image_height))
        ymaxs.append(float(box['y_max'] / image_height))
        classes_text.append(box['label'].encode())
        classes.append(int(LABEL_DICT[box['label']]))
    
    tf_example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': dataset_util.int64_feature(image_height),
        'image/width': dataset_util.int64_feature(image_width),
        'image/filename': dataset_util.bytes_feature(filename),
        'image/source_id': dataset_util.bytes_feature(filename),
        'image/encoded': dataset_util.bytes_feature(encoded_image),
        'image/format': dataset_util.bytes_feature(image_format),
        'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
        'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
        'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
        'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
        'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
        'image/object/class/label': dataset_util.int64_list_feature(classes),
        }))
    return tf_example


def main(_):
    writer = tf.python_io.TFRecordWriter(FLAGS.output_path)

    # BOSCH
    #INPUT_YAML = "data/TFRecordCreation/BOSCH_addition_train.yaml"
    #examples = yaml.load(open(INPUT_YAML, 'rb').read())
    
    # LISA
    INPUT_YAML = "data/TFRecordCreation/LISA_dayTrain.yaml"
    examples = yaml.load(open(INPUT_YAML, 'rb').read())
    n_examples = len(examples)
    print("Loaded ", len(examples), "examples")
    
    for i in range(len(examples)):
        examples[i]['path'] = os.path.abspath(os.path.join(os.path.dirname(INPUT_YAML), examples[i]['path']))
        examples[i]['source'] = 'LISA'
    
    for counter, example in enumerate(examples):
        tf_example = create_tf_example(example)
        writer.write(tf_example.SerializeToString())
        
        if counter % 10 == 0:
            print("Percent done", (counter/n_examples)*100)
        counter += 1
    
    writer.close()


if __name__ == '__main__':
  tf.app.run()
