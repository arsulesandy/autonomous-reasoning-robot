#!/usr/bin/env python3
import json
import rospy
from std_msgs.msg import String


def try_import_sklearn():
    try:
        from sklearn.metrics import confusion_matrix, accuracy_score  # type: ignore
        return confusion_matrix, accuracy_score
    except Exception:
        return None, None


CONFUSION_FN, ACC_FN = try_import_sklearn()


def flatten_confusion(data):
    """Convert nested dict {actual:{pred:count}} into sorted label list and 2D matrix."""
    labels = sorted(set(data.keys()) | {p for preds in data.values() for p in preds.keys()})
    label_index = {l: i for i, l in enumerate(labels)}
    matrix = [[0 for _ in labels] for _ in labels]
    for actual, preds in data.items():
        for pred, count in preds.items():
            matrix[label_index[actual]][label_index[pred]] = count
    return labels, matrix


def on_metrics(msg):
    if not msg.data:
        return
    try:
        data = json.loads(msg.data)
    except Exception as exc:
        rospy.logwarn_throttle(5.0, "Failed to parse metrics json: %s", exc)
        return

    labels, matrix = flatten_confusion(data)
    if not labels:
        return

    rospy.loginfo("Confusion labels: %s", labels)
    rospy.loginfo("Counts matrix: %s", matrix)

    if CONFUSION_FN and ACC_FN:
        y_true = []
        y_pred = []
        for i, actual in enumerate(labels):
            for j, pred in enumerate(labels):
                count = matrix[i][j]
                y_true.extend([actual] * count)
                y_pred.extend([pred] * count)
        if y_true:
            cm = CONFUSION_FN(y_true, y_pred, labels=labels)
            acc = ACC_FN(y_true, y_pred)
            rospy.loginfo("sklearn confusion matrix:\n%s", cm)
            rospy.loginfo("sklearn accuracy: %.3f", acc)
        else:
            rospy.loginfo("No samples yet for sklearn metrics")
    else:
        rospy.logwarn_throttle(10.0, "sklearn not available; showing raw counts only.")


def main():
    rospy.init_node("learning_metrics")
    rospy.Subscriber("learning/confusion_json", String, on_metrics, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
