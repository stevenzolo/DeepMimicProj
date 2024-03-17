import tensorflow as tf
import learning.tf_util as TFUtil

NAME = "fc_2layers_gated_1024units"


def build_net(input_tfs, reuse=False):
    print('gated net is built')
    layers = [1024, 512]
    gate_common_layers = [128]
    gate_layers = [64]
    activation = tf.nn.relu
    weight_init = tf.contrib.layers.xavier_initializer()
    bias_scale_kernel_init = tf.contrib.layers.xavier_initializer()

    gate_param_tf = input_tfs[-1]  # this should be the goal

    goal_size = input_tfs[-1].shape[1]
    print('size of goal', goal_size)

    with tf.variable_scope("gate_common", reuse=reuse):
        gate_common_tf = TFUtil.fc_net(gate_param_tf, gate_common_layers,
                                       activation=activation, reuse=reuse)
        gate_common_tf = activation(gate_common_tf)

    input_tf = tf.concat(axis=-1, values=input_tfs)

    curr_tf = input_tf

    # added by Yifan
    print('curr_tf at the beginning: ', curr_tf)
    curr_state = curr_tf[:, 0:226]
    print('curr_state: ', curr_state)
    curr_map = curr_tf[:, 226:1250]
    print('curr_map: ', curr_map)
    curr_goal = input_tfs[-1]
    print('curr_goal: ', curr_goal)
    curr_map = tf.reshape(curr_map, [-1, 32, 32, 1])
    curr_map = TFUtil.height_map_net(curr_map)
    curr_map = tf.squeeze(curr_map, [1, 2])
    curr_tf = tf.concat([curr_state, curr_map, curr_goal], axis=-1)
    print('curr_tf before fc: ', curr_tf)

    for i, size in enumerate(layers):
        with tf.variable_scope(str(i), reuse=reuse):
            curr_tf = tf.layers.dense(inputs=curr_tf,
                                      units=size,
                                      kernel_initializer=weight_init,
                                      activation=None)

        with tf.variable_scope("gate{:d}".format(i), reuse=reuse):
            if (len(gate_layers) == 0):
                curr_gate_h_tf = gate_common_tf
            else:
                curr_gate_h_tf = TFUtil.fc_net(input=gate_common_tf,
                                               layers_sizes=gate_layers,
                                               activation=activation,
                                               reuse=reuse)
                curr_gate_h_tf = activation(curr_gate_h_tf)

            curr_gate_bias_tf = tf.layers.dense(inputs=curr_gate_h_tf,
                                                units=size,
                                                kernel_initializer=bias_scale_kernel_init,
                                                bias_initializer=tf.zeros_initializer(),
                                                activation=None)

            curr_gate_scale_tf = tf.layers.dense(inputs=curr_gate_h_tf,
                                                 units=size,
                                                 kernel_initializer=bias_scale_kernel_init,
                                                 bias_initializer=tf.zeros_initializer(),
                                                 activation=None)
            curr_gate_scale_tf = 2 * tf.sigmoid(curr_gate_scale_tf)

        curr_tf = curr_gate_scale_tf * curr_tf + curr_gate_bias_tf
        curr_tf = activation(curr_tf)

    return curr_tf