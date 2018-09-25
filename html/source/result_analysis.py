from __future__ import print_function
from __future__ import print_function
from __future__ import print_function

from mongoclient import MongoWrapper
from bson.objectid import ObjectId

import itertools
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import pylab as pl
from mpl_toolkits.mplot3d import Axes3D


import operator
from operator import add


from sklearn.metrics import confusion_matrix
# from sklearn.metrics import classification_report
from sklearn.metrics import accuracy_score
from sklearn.metrics import precision_recall_fscore_support

# ===============================================================================
# cm_oneshot_shape = confusion_matrix(res_analysis.gt_shape_oneshot,
#                                     res_analysis.predicted_shape_oneshot,
#                                     list(res_analysis.shape_labels))
# print('OneShot Shape')
# print(cm_oneshot_shape)
#
# cm_amortized_shape = confusion_matrix(res_analysis.gt_shape_amortized,
#                                       res_analysis.predicted_shape_amortized,
#                                       list(res_analysis.shape_labels))
# print('Amortized Shape')
# print(cm_amortized_shape)
#
# # ===============================================================================
# cm_oneshot_color = confusion_matrix(res_analysis.gt_color_oneshot,
#                                     res_analysis.predicted_color_oneshot,
#                                     labels=list(res_analysis.color_labels))
# print('OneShot Color')
# print(cm_oneshot_color)
#
# cm_amortized_color = confusion_matrix(res_analysis.gt_color_amortized,
#                                       res_analysis.predicted_color_amortized,
#                                       labels=list(res_analysis.color_labels))
# print('Amortized Color')
# print(cm_amortized_color)

# print('OneShot CLASS accuracy: ',
#       accuracy_score(res_analysis.gt_class_oneshot, res_analysis.predicted_class_oneshot))
# print(classification_report(res_analysis.gt_class_oneshot,
#                             res_analysis.predicted_class_oneshot,
#                             list(res_analysis.class_labels)))
#
# print('Amortized CLASS Accuracy: ',
#       accuracy_score(res_analysis.gt_class_amortized, res_analysis.predicted_class_amortized))
# print(classification_report(res_analysis.gt_class_amortized,
#                             res_analysis.predicted_class_amortized,
#                             list(res_analysis.class_labels)))

# print('OneShot SHAPE accuracy: ',
#       accuracy_score(res_analysis.gt_shape_oneshot, res_analysis.predicted_shape_oneshot))
# print(classification_report(res_analysis.gt_shape_oneshot,
#                             res_analysis.predicted_shape_oneshot,
#                             list(res_analysis.shape_labels)))
#
# print('Amortized SHAPE accuracy: ',
#       accuracy_score(res_analysis.gt_shape_amortized, res_analysis.predicted_shape_amortized))
# print(classification_report(res_analysis.gt_shape_amortized,
#                             res_analysis.predicted_shape_amortized,
#                             list(res_analysis.shape_labels)))
#
# print('OneShot COLOR accuracy: ',
#       accuracy_score(res_analysis.gt_color_oneshot, res_analysis.predicted_color_oneshot))
# print(classification_report(res_analysis.gt_color_oneshot,
#                             res_analysis.predicted_color_oneshot,
#                             list(res_analysis.color_labels)))
#
# print('Amortized COLOR accuracy: ',
#       accuracy_score(res_analysis.gt_color_amortized, res_analysis.predicted_color_amortized))
# print(classification_report(res_analysis.gt_color_amortized,
#                             res_analysis.predicted_color_amortized,
#                             list(res_analysis.color_labels)))

# plt.figure()
# plot_confusion_matrix(cm_oneshot_class, list(res_analysis.class_labels),
#                       normalize=True,
#                       with_labels=False,
#                       title='CM one shot')
# #
# plt.figure()
# plot_confusion_matrix(cm_amortized_class, list(res_analysis.class_labels),
#                       normalize=True,
#                       with_labels=False,
#                       title='CM Amortized')
#
# plt.figure()
# plot_confusion_matrix(cm_oneshot_shape, classes=res_analysis.shape_labels,
#                       normalize=True,
#                       with_labels=True,
#                       title='CM Oneshot')
# plt.figure()
# plot_confusion_matrix(cm_amortized_shape, classes=res_analysis.shape_labels,
#                       normalize=True,
#                       with_labels=True,
#                       title='CM Amortized')
#
# plt.figure()
# plot_confusion_matrix(cm_oneshot_color, classes=res_analysis.color_labels,
#                       normalize=True,
#                       with_labels=True,
#                       title='CM Oneshot')
# plt.figure()
# plot_confusion_matrix(cm_amortized_color, classes=res_analysis.color_labels,
#                       normalize=True,
#                       with_labels=True,
#                       title='CM Amortized')
# plt.show()


key_values = {"shape": ["box", "round", "flat"],
              "color": ["red", "green", "blue", "white", "black", "yellow", "cyan", "magenta", "grey"],
              "type": ["CupEcoOrange", "RedMetalPlateWhiteSpeckles",
                       "BluePlasticFork", "BluePlasticKnife", "RedPlasticKnife", "RedPlasticFork",
                       "BlueMetalPlateWhiteSpeckles", "SiggBottle", "ElBrygCoffee",
                       "KelloggsCornFlakes", "ReineButterMilch", "PfannerPfirsichIcetea",
                       "VollMilch", "SojaMilch", "MeerSalz", "BluePlasticBowl", "SlottedSpatula",
                       "TomatoAlGustoBasilikum", "SeverinPancakeMaker", "MondaminPancakeMix",
                       "NesquikCereal", "TomatoSauceOroDiParma", "SpitzenReis", "LinuxCup"]}

ground_truth = {"CupEcoOrange": {"shape": ["round"],
                                 "color": ["yellow", "red"]},
                "RedMetalPlateWhiteSpeckles": {"shape": ["round", "flat"],
                                               "color": ["red"]},
                "BluePlasticFork": {"shape": ["flat"],
                                    "color": ["blue"]},
                "BluePlasticKnife": {"shape": ["flat"],
                                     "color": ["blue"]},
                "RedPlasticKnife": {"shape": ["flat"],
                                    "color": ["red"]},
                "RedPlasticFork": {"shape": ["flat"],
                                   "color": ["red"]},
                "BlueMetalPlateWhiteSpeckles": {"shape": ["flat", "round"],
                                                "color": ["blue"]},
                "SiggBottle": {"shape": ["round"],
                               "color": ["white", "green"]},
                "ElBrygCoffee": {"shape": ["box"],
                                 "color": ["red", "green"]},
                "KelloggsCornFlakes": {"shape": ["box"],
                                       "color": ["yellow"]},
                "ReineButterMilch": {"shape": ["round"],
                                     "color": ["white"]},
                "PfannerPfirsichIcetea": {"shape": ["box"],
                                          "color": ["blue", "white"]},
                "VollMilch": {"shape": ["box"],
                              "color": ["blue", "white"]},
                "SojaMilch": {"shape": ["box"],
                              "color": ["green", "white"]},
                "MeerSalz": {"shape": ["round"],
                             "color": ["blue"]},
                "BluePlasticBowl": {"shape": ["round"],
                                    "color": ["blue"]},
                "SlottedSpatula": {"shape": ["flat"],
                                   "color": ["black"]},
                "TomatoAlGustoBasilikum": {"shape": ["round"],
                                           "color": ["yellow"]},
                "SeverinPancakeMaker": {"shape": ["round"],
                                        "color": ["black"]},
                "MondaminPancakeMix": {"shape": ["round"],
                                       "color": ["blue", "yellow", "white"]},
                "NesquikCereal": {"shape": ["box"],
                                  "color": ["yellow"]},
                "TomatoSauceOroDiParma": {"shape": ["round"],
                                          "color": ["red"]},
                "SpitzenReis": {"shape": ["box"],
                                "color": ["red"]},
                "LinuxCup": {"shape": ["round"],
                             "color": ["blue", "white"]}}


def plot_confusion_matrix(cm, classes,
                          with_labels=False,
                          normalize=True,
                          title='Confusion matrix',
                          cmap=plt.cm.jet,
                          ):
    """
    This function prints and plots the confusion matrix.
    Normalization can be applied by setting `normalize=True`.
    """
    if normalize:
        cm = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]
        print("Normalized confusion matrix")
    else:
        print('Confusion matrix, without normalization')

    # print(cm)

    plt.imshow(cm, interpolation='nearest', cmap=cmap)
    plt.title(title)
    plt.colorbar()
    if with_labels:
        tick_marks = np.arange(len(classes))
        plt.xticks(tick_marks, classes, rotation=90)
        plt.yticks(tick_marks, classes)

    fmt = '.2f' if normalize else 'd'
    thresh = cm.max() / 2.
    for i, j in itertools.product(range(cm.shape[0]), range(cm.shape[1])):
        if cm[i,j] != 0:
            plt.text(j, i, format(cm[i, j], fmt),
                 horizontalalignment="center",
                 verticalalignment='center',
                 color="white" )#if cm[i, j] > thresh else "black")

    plt.tight_layout()
    plt.ylabel('True label')
    plt.xlabel('Predicted label')


class RSResultAnalysis(object):
    def __init__(self, db_name):
        self.mw = MongoWrapper(db_name)

        self.predicted_shape_oneshot = []
        self.gt_shape_oneshot = []
        self.predicted_shape_amortized = []
        self.gt_shape_amortized = []

        self.predicted_color_oneshot = []
        self.gt_color_oneshot = []
        self.predicted_color_amortized = []
        self.gt_color_amortized = []

        self.predicted_class_oneshot = []
        self.gt_class_oneshot = []
        self.predicted_class_amortized = []
        self.gt_class_amortized = []

        self.class_labels = set()
        self.shape_labels = set()
        self.color_labels = set()

        self.obj_hyp_count = 0

        self.cut_off_confidence = 0.55
        self.amortization_coeff = 5

    def set_cutoff_confidence(self, value):
        self.cut_off_confidence = value

    def set_amortization_coeff(self, value):
        self.amortization_coeff = value

    def analise_results(self):
        cas_iter = self.mw.db.cas.find()
        for cas in cas_iter:
            # print(cas['_timestamp'])
            scene_cursor = self.mw.db.scene.find({'_id': cas['scene']})
            for scene in scene_cursor:
                for ident in scene['identifiables']:
                    self.obj_hyp_count = self.obj_hyp_count + 1
                    shape_hyp = []
                    color_hyp = []
                    classification_hyp = ''
                    gr_truth = ''
                    for annot in ident['annotations']:
                        if annot['_type'] == 'rs.annotation.Detection':
                            if annot['confidence'] > 0.6:
                                classification_hyp = annot['name']
                        if annot['_type'] == 'rs.annotation.GroundTruth':
                            gr_truth = annot['classificationGT']['classname']
                        if annot['_type'] == 'rs.annotation.Shape':
                            shape_hyp.append(annot['shape'])
                        if annot['_type'] == 'rs.annotation.SemanticColor':
                            for i, c in enumerate(annot['color']):
                                if annot['ratio'][i] > 0.3:
                                    color_hyp.append(c)
                    [shape_obj, color_obj, class_obj] = self.get_results_with_amortization(str(ident['_id']))

                    if gr_truth is not '':
                        # CLASS
                        if classification_hyp is not '':
                            self.predicted_class_oneshot.append(str(classification_hyp))
                            self.gt_class_oneshot.append(str(gr_truth))
                            self.class_labels.add(str(gr_truth))

                        if class_obj:
                            best_amortized = max(class_obj.iteritems(), key=operator.itemgetter(1))[0]

                            self.predicted_class_amortized.append(str(best_amortized))
                            self.gt_class_amortized.append(str(gr_truth))

                        # SHAPE
                        gt_shape = ground_truth[gr_truth]['shape']
                        if len(shape_hyp) > 0:
                            # print('One Shot: ', shape_hyp)
                            for s in shape_hyp:
                                self.predicted_shape_oneshot.append(s)
                                try:
                                    gt_shape.index(s)
                                    self.gt_shape_oneshot.append(s)
                                except ValueError:
                                    self.gt_shape_oneshot.append(gt_shape[0])
                                self.shape_labels.add(s)

                        # amortized shape values
                        if len(shape_obj) > 0:
                            best_amortized_count = max(shape_obj.iteritems(),
                                                       key=operator.itemgetter(1))[1]
                            top_results = dict((k, v) for k, v in shape_obj.items() if
                                               v >= best_amortized_count - best_amortized_count * 0.05)
                            for s, score in top_results.items():
                                self.predicted_shape_amortized.append(s)
                                try:
                                    gt_shape.index(s)
                                    self.gt_shape_amortized.append(s)
                                except ValueError:
                                    self.gt_shape_amortized.append(gt_shape[0])

                        # COLOR
                        gt_color = ground_truth[gr_truth]['color']
                        if len(color_hyp) > 0:
                            # print('One Shot: ', color_hyp)
                            for color in color_hyp:
                                self.predicted_color_oneshot.append(color)
                                try:
                                    gt_color.index(color)
                                    self.gt_color_oneshot.append(color)
                                except ValueError:
                                    self.gt_color_oneshot.append(gt_color[0])
                                self.color_labels.add(color)

                        if len(color_obj) > 0:
                            best_amortized_count = max(color_obj.iteritems(),
                                                       key=operator.itemgetter(1))[1]
                            top_results = dict((k, v) for k, v in color_obj.items() if
                                               v >= best_amortized_count - best_amortized_count * 0.05)
                            for c, score in top_results.items():
                                self.predicted_color_amortized.append(c)
                                try:
                                    gt_color.index(c)
                                    self.gt_color_amortized.append(c)
                                except ValueError:
                                    self.gt_color_amortized.append(gt_color[0])

    def get_results_with_amortization(self, hyp_id):
        obj = self.mw.db.persistent_objects.find({'clusters': hyp_id})
        if obj.count() == 0:
            return [{}, {}, {}]
        idx = obj[0]['clusters'].index(hyp_id)

        shape_hyp = {}
        color_hyp = {}
        type_hyp = {}

        counter = 0
        # sum_conf = 0.0
        while idx >= 0 and counter < self.amortization_coeff:
            c_id = obj[0]['clusters'][idx]
            hypotheses = self.mw.db.scene.find({'identifiables._id': ObjectId(c_id)},
                                               {'_id': 0, 'identifiables._id.$': 1, 'timestamp': 1})[0]
            # print hypotheses
            for annot in hypotheses['identifiables'][0]['annotations']:
                if annot['_type'] == 'rs.annotation.Detection':
                    if idx == obj[0]['clusters'].index(hyp_id) and annot['confidence']>0.65:
                        type_hyp[annot['name']] = type_hyp.get(annot['name'], [0, 0.0])
                        type_hyp[annot['name']][0] = type_hyp[annot['name']][0] + 1
                        type_hyp[annot['name']][1] = annot['confidence']
                        # sum_conf = sum_conf + annot['confidence']
                    elif annot['confidence'] > self.cut_off_confidence:
                        type_hyp[annot['name']] = type_hyp.get(annot['name'], [0, 0.0])
                        type_hyp[annot['name']][0] = type_hyp[annot['name']][0] + 1
                        type_hyp[annot['name']][1] = annot['confidence'] #type_hyp[annot['name']][1] + annot['confidence']
                        # sum_conf = sum_conf + annot['confidence']
                if annot['_type'] == 'rs.annotation.Shape':
                    shape_hyp[annot['shape']] = shape_hyp.get(annot['shape'], 0) + 1
                if annot['_type'] == 'rs.annotation.SemanticColor':
                    for i, c in enumerate(annot['color']):
                        if annot['ratio'][i] > 0.3:
                            color_hyp[c] = color_hyp.get(c, 0) + 1
            idx = idx - 1
            counter = counter + 1
        # for v, key in type_hyp.items():
        #     type_hyp[v] = key[1] / sum_conf
        #     # key[1] = key[1]/key[0]
        return [shape_hyp, color_hyp, type_hyp]


def color_y_axis(ax, color):
    """Color your axes."""
    for t in ax.get_yticklabels():
        t.set_color(color)
    return None


def two_scales(a1, X, Y_Hypotheses, Y_Accuracy,conf_threshold):

    a2 = a1.twinx()

    linestyles = ['-', '--', '-.', ':']
    idx = 0
    for Y in Y_Hypotheses:
        a1.plot(X,Y, color='b',linestyle=linestyles[idx])
        idx = idx+1

    label ='Amortization coefficient (conf_threshold = ' + str(conf_threshold) + ')'
    a1.set_xlabel(label)
    a1.set_ylabel('% of hypotheses')

    idx = 0
    for Y in Y_Accuracy:
        a2.plot(X, Y, color='r',linestyle = linestyles[idx])
        idx = idx + 1

    a2.set_ylabel('Accuracy')

    return a1, a2


if __name__ == "__main__":

    episodes = ['PnP09ObjSymbolicGTFixed',
                'PnP15ObjSymbolicGTFixed',
                'PnP20ObjSymbolicGTFixed',
                'PnP25ObjSymbolicGTFixed']

    amortization_coefficients  = range(1,20,1)
    confidence_thresholds = pl.frange(0.6, 0.8, 0.02)

    # confidence_thresholds = [0.6]
    # amortization_coefficients = [13]
    print(confidence_thresholds)
    for i,v in enumerate(confidence_thresholds):
    	temp = float("{0:.2f}".format(v))
        confidence_thresholds[i]=temp
    print(confidence_thresholds)
    # accuracies = [][]
    all_accuracies = [] #np.zeros((len(cutoffs), len(coeffs)))
    all_hyp_ratios = [] #np.zeros((len(cutoffs), len(coeffs)))

    heat_map_acc = np.zeros((len(confidence_thresholds), len(amortization_coefficients)))
    heat_map_hyp = np.zeros((len(confidence_thresholds), len(amortization_coefficients)))

    scatter_coeffs = []
    scatter_cutoffs = []
    print(all_accuracies)

    for ce_idx, ce in enumerate(confidence_thresholds):
        # print(ce_idx)
        average_oneshot_accuracy = []
        average_amortized_accuracy = []

        average_oneshot_precision = []
        average_amortized_precision = []

        oneshot_hypotheses_classified = []
        amortized_hypotheses_classified = []
        atotal_nr_of_hypotheses = []
        for c_idx, c in enumerate(amortization_coefficients):
            print('coeff: ', c)
            print('cutoffs:',ce)

            gt_class_oneshot = []
            gt_class_amortized = []
            predicted_class_oneshot = []
            predicted_class_amortized = []
            total_nr_of_hypotheses = 0
            class_labels = set()

            for e in episodes:
                res_analysis = RSResultAnalysis(e)
                res_analysis.set_amortization_coeff(c)
                res_analysis.set_cutoff_confidence(ce)
                res_analysis.analise_results()

                gt_class_oneshot.extend(res_analysis.gt_class_oneshot)
                predicted_class_oneshot.extend(res_analysis.predicted_class_oneshot)

                gt_class_amortized.extend(res_analysis.gt_class_amortized)
                predicted_class_amortized.extend(res_analysis.predicted_class_amortized)

                acc = accuracy_score(res_analysis.gt_class_oneshot, res_analysis.predicted_class_oneshot)
                [p, r, f, s] = precision_recall_fscore_support(res_analysis.gt_class_oneshot,
                                                               res_analysis.predicted_class_oneshot,
                                                               labels=list(res_analysis.class_labels),
                                                               average='weighted')
                print('OneShot')
                print(e, ':', "Precision:", p, ' Recall: ', r, ' Accuracy: ', acc)
                acc = accuracy_score(res_analysis.gt_class_amortized, res_analysis.predicted_class_amortized)
                [p, r, f, s] = precision_recall_fscore_support(res_analysis.gt_class_amortized,
                                                               res_analysis.predicted_class_amortized,
                                                               labels=list(res_analysis.class_labels),
                                                               average='weighted')
                print('Amortized')
                print(e, ':', "Precision:", p, ' Recall: ', r, ' Accuracy: ', acc)
                print(e, 'Total number of hyp:',res_analysis.obj_hyp_count)
                total_nr_of_hypotheses = total_nr_of_hypotheses + res_analysis.obj_hyp_count

                class_labels = class_labels | res_analysis.class_labels

            atotal_nr_of_hypotheses.append(total_nr_of_hypotheses)
            acc = accuracy_score(gt_class_oneshot, predicted_class_oneshot)
            [p, r, f, s] = precision_recall_fscore_support(gt_class_oneshot,
                                                           predicted_class_oneshot,
                                                           labels=list(class_labels),
                                                           average='weighted')
            print('OneShot')
            print(e, ':', "Precision:", p, ' Recall: ', r, ' Accuracy: ', acc)
            average_oneshot_accuracy.append(acc)
            average_oneshot_precision.append(p)
            oneshot_hypotheses_classified.append(len(gt_class_oneshot)/float(total_nr_of_hypotheses))


            acc = accuracy_score(gt_class_amortized, predicted_class_amortized)

            [p, r, f, s] = precision_recall_fscore_support(gt_class_amortized,
                                                           predicted_class_amortized,
                                                           labels=list(class_labels),
                                                           average='weighted')
            print('Amortized')
            print (e,':',"Precision:",p,' Recall: ',r,' Accuracy: ',acc)
            average_amortized_accuracy.append(acc)
            average_amortized_precision.append(p)

            heat_map_acc[ce_idx][c_idx] = acc
            heat_map_hyp[ce_idx][c_idx] = len(gt_class_amortized) / float(total_nr_of_hypotheses)
            all_accuracies.append(acc)
            all_hyp_ratios.append(len(gt_class_amortized) / float(total_nr_of_hypotheses))

            scatter_coeffs.append(c)
            scatter_cutoffs.append(ce)

            amortized_hypotheses_classified.append(len(gt_class_amortized)/float(total_nr_of_hypotheses))

            # cm_oneshot_class = confusion_matrix(res_analysis.gt_class_oneshot,
            #                                     res_analysis.predicted_class_oneshot,
            #                                     list(res_analysis.class_labels))
            #
            # cm_amortized_class = confusion_matrix(res_analysis.gt_class_amortized,
            #                                       res_analysis.predicted_class_amortized,
            #                                       list(res_analysis.class_labels))
            #
            # plt.figure()
            # plot_confusion_matrix(cm_oneshot_class, list(res_analysis.class_labels),
            #                       normalize=False,
            #                       with_labels=False,
            #                       title='CM one shot')
            # plt.savefig('cm_oneshot_class.png')
            # plt.figure()
            # plot_confusion_matrix(cm_amortized_class, list(res_analysis.class_labels),
            #                       normalize=False,
            #                       with_labels=False,
            #                       title='CM Amortized')
            # plt.savefig('cm_oneshot_class.png')

        print("OneShot accuracies: ", average_oneshot_accuracy)
        print("OneShot instances:", oneshot_hypotheses_classified)
        print("Amortized accuracies:", average_amortized_accuracy)
        print("Amortized instances:", amortized_hypotheses_classified)

        print("Total nr of hyp:", atotal_nr_of_hypotheses)

        fig, ax = plt.subplots()
        ax1, ax2 = two_scales(ax, amortization_coefficients,
                              [amortized_hypotheses_classified,oneshot_hypotheses_classified],
                              [average_amortized_accuracy, average_oneshot_accuracy], ce)

        color_y_axis(ax1, 'b')
        color_y_axis(ax2, 'r')
        plt_name = 'a_coeff_acc_hyp_threshold_'+str(ce)+'.png'

        line1, = plt.plot([], label="OneShot", linestyle='--', color='b')
        line2, = plt.plot([], label="Amortized", linestyle='-', color='b')

        plt.legend(handles=[line1, line2], loc=2)
        # line1, = fig.plot([1, 2, 3], label="OneShot", linestyle='--')
        # line2, = fig.plot([3, 2, 1], label="Amortized", linestyle='-')

        # Create a legend for the first line.
        # first_legend = plt.legend(handles=[line1], loc=1)
        # second_legend = plt.legend(handles=[line2], loc=2)
        # Add the legend manually to the current Axes.
        # ax1 = fig.gca().add_artist(first_legend)

        # Create another legend for the second line.
        # fig.legend(handles=[line2], loc=4)

        plt.savefig(plt_name)
        # plt.show()

    print(all_accuracies)
    print(all_hyp_ratios)
    grid_sum = map(add, all_hyp_ratios, all_accuracies)


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(scatter_coeffs, scatter_cutoffs, all_accuracies, c='b', marker='o')
    ax.scatter(scatter_coeffs, scatter_cutoffs, all_hyp_ratios, c='r', marker='s')


    ax.set_xlabel('Amortization coefficient(ac)')
    ax.set_ylabel('Confidence threshold (cf)')
    ax.set_zlabel('Accuracy/ [%] of hypotheses')

    scatter1_proxy = mpl.lines.Line2D([0], [0], linestyle="none", c='b', marker='o')
    scatter2_proxy = mpl.lines.Line2D([0], [0], linestyle="none", c='r', marker='s')
    ax.legend([scatter1_proxy, scatter2_proxy], ['Accuracy', '% obj hypt'], numpoints=1)
    plt.savefig('gs_3d.png')

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(scatter_coeffs, scatter_cutoffs, grid_sum, c='b', marker='o')
    ax.set_xlabel('Amortization coefficient(ac)')
    ax.set_ylabel('Confidence threshold (cf)')
    ax.set_zlabel('Sum')
    ax.legend([scatter1_proxy], ['Accuracy + % obj hypt'], numpoints=1)

    plt.savefig('gs_3d_sum.png')

    plt.figure(figsize=(8, 6))
    plt.subplots_adjust(left=.2, right=0.95, bottom=0.15, top=0.95)
    plt.imshow(heat_map_hyp, interpolation='nearest', cmap=plt.cm.hot)
    plt.ylabel('Confidence threshold (cf)')
    plt.xlabel('Amortization coefficient(ac)')
    plt.colorbar()
    plt.yticks(np.arange(len(confidence_thresholds)), confidence_thresholds)
    plt.xticks(np.arange(len(amortization_coefficients)), amortization_coefficients)
    plt.title('Grid Search % of Hypotheses annotated')

    plt.savefig('gs_hypotheses.png')

    plt.figure(figsize=(8, 6))
    plt.subplots_adjust(left=.2, right=0.95, bottom=0.15, top=0.95)
    plt.imshow(heat_map_acc, interpolation='nearest', cmap=plt.cm.hot)
    plt.ylabel('Confidence threshold (cf)')
    plt.xlabel('Amortization coefficient(ac)')
    plt.colorbar()
    plt.yticks(np.arange(len(confidence_thresholds)), confidence_thresholds)
    plt.xticks(np.arange(len(amortization_coefficients)), amortization_coefficients)

    plt.title('Grid Search Accuracy')
    plt.savefig('gs_accuracy.png')

    print(grid_sum)
    arr = heat_map_acc + heat_map_hyp
    row = np.where(arr == arr.max())[0][0]
    col = np.where(arr == arr.max())[1][0]
    res_coeff =  amortization_coefficients[np.where(arr == arr.max())[1][0]]
    res_threshold =  confidence_thresholds[np.where(arr == arr.max())[0][0]]
     
    print('Chosen Accuracy: ',heat_map_acc[row][col] )
    print('% of hypotheses: ',heat_map_hyp[row][col] )
   
    print('MAX VALUE IS: ', max(grid_sum), ' with params: ', res_coeff, ' threshold: ',res_threshold)  

    plt.show()



    # plt.figure()
    # plt.plot(coeffs, amortized_hypotheses_classified)
    # plt.plot(coeffs, oneshot_hypotheses_classified)
    #
    # plt.figure()
    # plt.plot(coeffs, average_amortized_accuracy)
    # plt.plot(coeffs, average_oneshot_accuracy)



    # PLOTTING

    # plt.figure()
    # plot_confusion_matrix(cm_oneshot_class, list(class_labels),
    #                       normalize=False,
    #                       with_labels=True,
    #                       title='CM one shot')
    # #
    # plt.figure()
    # plot_confusion_matrix(cm_amortized_class, list(class_labels),
    #                       normalize=False,
    #                       with_labels=True,
    #                       title='CM Amortized')
    # plt.show()
