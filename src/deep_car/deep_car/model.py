from __future__ import division
import tensorflow as tf
import numpy as np


class Model:
    """Build the neural network"""

    def __init__(self, input_shape, reuse=False):
        with tf.variable_scope("deep_car", reuse=reuse):
            # inputs
            self.image = tf.placeholder(tf.float32, shape=input_shape, name='image')
            self.steering_abs_true = tf.placeholder(tf.float32, shape=[None, 1],
                                                    name='steering_abs_true')
            # training flag for batch norm
            self.training = tf.placeholder_with_default(False, shape=[], name='training')
            # ground truth
            self.setup_model()
            self.setup_optimizer()

    def setup_model(self):
        f = 32
        k = 5
        # TODO: fix batch normalization

        # data images are 48x64
        l = tf.layers.conv2d(self.image, f, 5, padding='same')
        l = tf.layers.batch_normalization(l, training=self.training)
        l = tf.nn.relu(l)
        l = tf.layers.max_pooling2d(l, 2, 2)

        # 24x32 after pooling
        l = tf.layers.conv2d(l, 2*f, k, padding='same')
        l = tf.layers.batch_normalization(l, training=self.training)
        l = tf.nn.relu(l)
        l = tf.layers.max_pooling2d(l, 2, 2)

        # 12x16
        l = tf.layers.conv2d(l, 4*f, k, padding='same')
        l = tf.layers.batch_normalization(l, training=self.training)
        l = tf.nn.relu(l)
        l = tf.layers.max_pooling2d(l, (2, 2), 2)

        l = tf.layers.conv2d(l, 4*f, k, padding='same')
        l = tf.layers.batch_normalization(l, training=self.training)
        l = tf.nn.relu(l)
        l = tf.layers.max_pooling2d(l, (2, 2), 2)

        # 6x8
        l = tf.layers.conv2d(l, 8*f, k, padding='same')
        l = tf.layers.batch_normalization(l, training=self.training)
        l = tf.nn.relu(l)
        l = tf.layers.average_pooling2d(l, (3, 4), 1)
        l = tf.contrib.layers.flatten(l)

        l = tf.layers.dense(l, 4*f)
        l = tf.layers.batch_normalization(l, training=self.training)
        self.steering_abs_pred = tf.identity(
            np.pi*tf.tanh(tf.layers.dense(l, 1)), name='steering_abs_pred')

    def setup_optimizer(self):
        self.steering_abs_loss = tf.identity(
           tf.reduce_mean((self.steering_abs_true - self.steering_abs_pred)**2)
        )

        opt = tf.train.AdamOptimizer(learning_rate=0.0003)
        update_ops = tf.get_collection(tf.GraphKeys.UPDATE_OPS)
        with tf.control_dependencies(update_ops):
            self.opt_op = opt.minimize(
                self.steering_abs_loss, name='opt')
