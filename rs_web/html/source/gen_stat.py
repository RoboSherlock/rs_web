
from __future__ import print_function

from mongoclient import MongoWrapper

import itertools
import numpy as np
import matplotlib.pyplot as plt


from sklearn.metrics import confusion_matrix
from sklearn.metrics import classification_report
from sklearn.metrics import accuracy_score

import argparse

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
        # plt.xticks(tick_marks, classes, rotation=90)
        plt.yticks(tick_marks, classes)

    fmt = '.2f' if normalize else 'd'
    thresh = cm.max() / 2.
    for i, j in itertools.product(range(cm.shape[0]), range(cm.shape[1])):
        if cm[i,j] > 0:
            plt.text(j, i, format(cm[i, j], fmt),
                     horizontalalignment="center",
                     color="white" if cm[i, j] > thresh else "black")

    plt.tight_layout()
    plt.ylabel('True label')
    plt.xlabel('Predicted label')


def color_y_axis(ax, color):
    """Color your axes."""
    for t in ax.get_yticklabels():
        t.set_color(color)
    return None


class RSResultAnalysis(object):
    def __init__(self, db_name):
        self.mw = MongoWrapper(db_name)

        self.predicted_class_real_data = []
        self.predicted_class_unreal_data = []
        self.gt_class = []
        self.class_labels = set()

        self.obj_hyp_count = 0

    def analise_results(self):
        cas_iter = self.mw.db.cas.find()
        for cas in cas_iter:
            # print(cas['_timestamp'])
            scene_cursor = self.mw.db.scene.find({'_id': cas['scene']})
            for scene in scene_cursor:
                for ident in scene['identifiables']:
                    self.obj_hyp_count = self.obj_hyp_count + 1
                    classification_hyp = ''
                    ue_classification_hyp = ''
                    gr_truth = ''
                    for annot in ident['annotations']:
                        if annot['_type'] == 'rs.annotation.Detection':
                            if annot['name'] == 'LionCerealBox' or annot['name'] == 'JodSalz':
                                continue
                            if annot['source'] == 'DeCafClassifier':
                                classification_hyp = annot['name']
                            if annot['source'] == 'DeCafClassifier_UE':
                                ue_classification_hyp = annot['name']
                        if annot['_type'] == 'rs.annotation.GroundTruth':
                            if annot['classificationGT']['classname'] == 'LionCerealBox' or annot['classificationGT']['classname'] == 'JodSalz':
                                continue
                            gr_truth = annot['classificationGT']['classname']

                    if gr_truth is not '':
                        # CLASS
                        if classification_hyp is not '':
                            self.predicted_class_real_data.append(str(classification_hyp))
                            self.predicted_class_unreal_data.append(str(ue_classification_hyp))
                            self.gt_class.append(str(gr_truth))
                            self.class_labels.add(str(gr_truth))

    def generate_cms(self):
        cm_real_data = confusion_matrix(self.gt_class,
                                        self.predicted_class_real_data,
                                        list(self.class_labels))

        cm_unreal_data = confusion_matrix(self.gt_class,
                                          self.predicted_class_unreal_data,
                                          list(self.class_labels))
        return [cm_real_data, cm_unreal_data]

    def get_accuracy(self):
        acr = accuracy_score(self.gt_class, self.predicted_class_real_data)
        acur = accuracy_score(self.gt_class, self.predicted_class_unreal_data)
        return [acr, acur]

    def print_reports(self):
        print(self.class_labels)
        print(classification_report(self.gt_class, self.predicted_class_real_data,
                                    list(self.class_labels)))
        print(classification_report(self.gt_class, self.predicted_class_unreal_data,
                                    list(self.class_labels)))


if __name__ == "__main__":

    episode = 'PnPBreakfastVariationResultWithUnreal'

    res_analysis = RSResultAnalysis(db_name=episode)
    res_analysis.analise_results()
    [cmr, cmur] = res_analysis.generate_cms()
    [acr, acur] = res_analysis.get_accuracy()

    print('Turn Table data accuracy', acr)
    print('Variations data accuracy ', acur)
    plt.figure()
    plot_confusion_matrix(cmr, list(res_analysis.class_labels),
                          normalize=False,
                          with_labels=True,
                          title='CM trained on turn table')
    plt.figure()
    plot_confusion_matrix(cmur, list(res_analysis.class_labels),
                          normalize=False,
                          with_labels=True,
                          title='CM trained on data from variations')

    res_analysis.print_reports()
    plt.show()

