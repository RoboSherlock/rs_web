from __future__ import print_function
from __future__ import print_function
from __future__ import print_function


from mongoclient import MongoWrapper
from bson.objectid import ObjectId

import itertools
import numpy as np
import matplotlib.pyplot as plt
import operator

from sklearn.metrics import confusion_matrix
from sklearn.metrics import classification_report
from sklearn.metrics import accuracy_score

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
    # for i, j in itertools.product(range(cm.shape[0]), range(cm.shape[1])):
    #     plt.text(j, i, format(cm[i, j], fmt),
    #              horizontalalignment="center",
    #              color="white" if cm[i, j] > thresh else "black")

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

    def analise_results(self):
        cas_iter = self.mw.db.cas.find()
        for cas in cas_iter:
            print(cas['_timestamp'])
            scene_cursor = self.mw.db.scene.find({'_id': cas['scene']})
            for scene in scene_cursor:
                for ident in scene['identifiables']:

                    shape_hyp = []
                    color_hyp = []
                    classification_hyp = ''
                    gr_truth = ''
                    for annot in ident['annotations']:
                        if annot['_type'] == 'rs.annotation.Detection':
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
                        print('=============================')
                        # CLASS
                        print('Ground Truth:', gr_truth)
                        print('Amortized: ', class_obj)
                        if classification_hyp is not '':
                            print('One shot: ', classification_hyp)
                            self.predicted_class_oneshot.append(str(classification_hyp))
                            self.gt_class_oneshot.append(str(gr_truth))
                            self.class_labels.add(str(gr_truth))
                        else:
                            print('One shot: NOT CONFIDENT')

                        if class_obj:
                            best_amortized = max(class_obj.iteritems(), key=operator.itemgetter(1))[0]
                            print('Best amortized: ', str(best_amortized))
                            self.predicted_class_amortized.append(str(best_amortized))
                            self.gt_class_amortized.append(str(gr_truth))

                        #SHAPE
                        gt_shape = ground_truth[gr_truth]['shape']
                        print('GT shape:', gt_shape)
                        if len(shape_hyp) > 0:
                            print('One Shot: ', shape_hyp)
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
                            print('Amortized:', shape_obj)
                            best_amortized_count = max(shape_obj.iteritems(),
                                                       key=operator.itemgetter(1))[1]
                            top_results = dict((k, v) for k, v in shape_obj.items() if
                                               v >= best_amortized_count - best_amortized_count * 0.05)
                            print('Top results', top_results)
                            for s, score in top_results.items():
                                self.predicted_shape_amortized.append(s)
                                try:
                                    gt_shape.index(s)
                                    self.gt_shape_amortized.append(s)
                                except ValueError:
                                    self.gt_shape_amortized.append(gt_shape[0])

                        # COLOR
                        gt_color = ground_truth[gr_truth]['color']
                        print('GT color: ', gt_color)
                        if len(color_hyp) > 0:
                            print('One Shot: ', color_hyp)
                            for color in color_hyp:
                                self.predicted_color_oneshot.append(color)
                                try:
                                    gt_color.index(color)
                                    self.gt_color_oneshot.append(color)
                                except ValueError:
                                    self.gt_color_oneshot.append(gt_color[0])
                                self.color_labels.add(color)

                        if len(color_obj) > 0:
                            print('Amortized:', color_obj)
                            best_amortized_count = max(color_obj.iteritems(),
                                                       key=operator.itemgetter(1))[1]
                            top_results = dict((k, v) for k, v in color_obj.items() if
                                               v >= best_amortized_count - best_amortized_count * 0.05)
                            print('Top results', top_results)
                            for c, score in top_results.items():
                                self.predicted_color_amortized.append(c)
                                try:
                                    gt_color.index(c)
                                    self.gt_color_amortized.append(c)
                                except ValueError:
                                    self.gt_color_amortized.append(gt_color[0])

                        print('===============')

    def get_results_with_amortization(self, hyp_id):
        obj = self.mw.db.persistent_objects.find({'clusters': hyp_id})
        if obj.count() == 0:
            return [{}, {}, {}]
        idx = obj[0]['clusters'].index(hyp_id)

        shape_hyp = {}
        color_hyp = {}
        type_hyp = {}

        counter = 0
        sum_conf = 0.0
        while idx >= 0 and counter < 5:
            c_id = obj[0]['clusters'][idx]
            hypotheses = self.mw.db.scene.find({'identifiables._id': ObjectId(c_id)},
                                               {'_id': 0, 'identifiables._id.$': 1, 'timestamp': 1})[0]
            # print hypotheses
            for annot in hypotheses['identifiables'][0]['annotations']:
                if annot['_type'] == 'rs.annotation.Detection':
                    type_hyp[annot['name']] = type_hyp.get(annot['name'], [0, 0.0])
                    type_hyp[annot['name']][0] = type_hyp[annot['name']][0] + 1
                    type_hyp[annot['name']][1] = type_hyp[annot['name']][1] + annot['confidence']
                    sum_conf = sum_conf + annot['confidence']
                if annot['_type'] == 'rs.annotation.Shape':
                    shape_hyp[annot['shape']] = shape_hyp.get(annot['shape'], 0) + 1
                if annot['_type'] == 'rs.annotation.SemanticColor':
                    for i, c in enumerate(annot['color']):
                        if annot['ratio'][i] > 0.3:
                            color_hyp[c] = color_hyp.get(c, 0) + 1
            idx = idx - 1
            counter = counter + 1

        for v,key in type_hyp.items():
            type_hyp[v] = key[1]/sum_conf
            # key[1] = key[1]/key[0]
        return [shape_hyp, color_hyp, type_hyp]


if __name__ == "__main__":
    res_analysis = RSResultAnalysis('PnP09  ObjSymbolicGTFixed')
    res_analysis.analise_results()

    cm_oneshot_class = confusion_matrix(res_analysis.gt_class_oneshot,
                                        res_analysis.predicted_class_oneshot,
                                        list(res_analysis.class_labels))

    cm_amortized_class = confusion_matrix(res_analysis.gt_class_amortized,
                                          res_analysis.predicted_class_amortized,
                                          list(res_analysis.class_labels))

    # ===============================================================================
    cm_oneshot_shape = confusion_matrix(res_analysis.gt_shape_oneshot,
                                        res_analysis.predicted_shape_oneshot,
                                        list(res_analysis.shape_labels))
    print('OneShot Shape')
    print(cm_oneshot_shape)

    cm_amortized_shape = confusion_matrix(res_analysis.gt_shape_amortized,
                                          res_analysis.predicted_shape_amortized,
                                          list(res_analysis.shape_labels))
    print('Amortized Shape')
    print(cm_amortized_shape)

    # ===============================================================================
    cm_oneshot_color = confusion_matrix(res_analysis.gt_color_oneshot,
                                        res_analysis.predicted_color_oneshot,
                                        labels=list(res_analysis.color_labels))
    print('OneShot Color')
    print(cm_oneshot_color)

    cm_amortized_color = confusion_matrix(res_analysis.gt_color_amortized,
                                          res_analysis.predicted_color_amortized,
                                          labels=list(res_analysis.color_labels))
    print('Amortized Color')
    print(cm_amortized_color)

    print('OneShot CLASS accuracy: ', accuracy_score(res_analysis.gt_class_oneshot, res_analysis.predicted_class_oneshot))
    print(classification_report(res_analysis.gt_class_oneshot,
                                res_analysis.predicted_class_oneshot,
                                list(res_analysis.class_labels)))

    print('Amortized CLASS Accuracy: ', accuracy_score(res_analysis.gt_class_amortized, res_analysis.predicted_class_amortized))
    print(classification_report(res_analysis.gt_class_amortized,
                                res_analysis.predicted_class_amortized,
                                list(res_analysis.class_labels)))

    print('OneShot SHAPE accuracy: ',
          accuracy_score(res_analysis.gt_shape_oneshot, res_analysis.predicted_shape_oneshot))
    print(classification_report(res_analysis.gt_shape_oneshot,
                                res_analysis.predicted_shape_oneshot,
                                list(res_analysis.shape_labels)))

    print('Amortized SHAPE accuracy: ',
          accuracy_score(res_analysis.gt_shape_amortized, res_analysis.predicted_shape_amortized))
    print(classification_report(res_analysis.gt_shape_amortized,
                                res_analysis.predicted_shape_amortized,
                                list(res_analysis.shape_labels)))

    print('OneShot COLOR accuracy: ',
          accuracy_score(res_analysis.gt_color_oneshot, res_analysis.predicted_color_oneshot))
    print(classification_report(res_analysis.gt_color_oneshot,
                                res_analysis.predicted_color_oneshot,
                                list(res_analysis.color_labels)))

    print('Amortized COLOR accuracy: ',
          accuracy_score(res_analysis.gt_color_amortized, res_analysis.predicted_color_amortized))
    print(classification_report(res_analysis.gt_color_amortized,
                                res_analysis.predicted_color_amortized,
                                list(res_analysis.color_labels)))

    plt.figure()
    plot_confusion_matrix(cm_oneshot_class, list(res_analysis.class_labels),
                          normalize=True,
                          with_labels=False,
                          title='CM one shot')
    #
    plt.figure()
    plot_confusion_matrix(cm_amortized_class, list(res_analysis.class_labels),
                          normalize=True,
                          with_labels=False,
                          title='CM Amortized')

    plt.figure()
    plot_confusion_matrix(cm_oneshot_shape, classes=res_analysis.shape_labels,
                          normalize=True,
                          with_labels=True,
                          title='CM Oneshot')
    plt.figure()
    plot_confusion_matrix(cm_amortized_shape, classes=res_analysis.shape_labels,
                          normalize=True,
                          with_labels=True,
                          title='CM Amortized')

    plt.figure()
    plot_confusion_matrix(cm_oneshot_color, classes=res_analysis.color_labels,
                          normalize=True,
                          with_labels=True,
                          title='CM Oneshot')
    plt.figure()
    plot_confusion_matrix(cm_amortized_color, classes=res_analysis.color_labels,
                          normalize=True,
                          with_labels=True,
                          title='CM Amortized')
    # plt.show()
