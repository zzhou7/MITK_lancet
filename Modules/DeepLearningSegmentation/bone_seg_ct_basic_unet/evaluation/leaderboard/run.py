import argparse
import json
import os
from collections import OrderedDict, defaultdict
import requests
import numpy as np
from flask import Blueprint, Flask, abort, render_template, request


REQUIRED_ENTRIES = ("name", "task", "results")
ABBREVIATIONS = {
    "False Positive Rate": "FPR",
    "false_positive_rate": "FPR",
    "Dice": "DICE",
    "dice": "DICE",
    "Jaccard": "JAC",
    "jaccard": "JAC",
    "Hausdorff Distance": "HD",
    "hausdorff_distance": "HD",
    "Hausdorff Distance 95": "HD95",
    "hausdorff_distance_95": "HD95",
    "Precision": "PREC",
    "precision": "PREC",
    "Recall": "REC",
    "recall": "REC",
    "Avg. Symmetric Surface Distance": "ASSD",
    "avg_surface_distance_symmetric": "ASSD",
    "Avg. Surface Distance": "ASD",
    "avg_surface_distance": "ASD",
    "Accuracy": "ACC",
    "accuracy": "ACC",
    "False Omission Rate": "FOR",
    "false_omission_rate": "FOR",
    "Negative Predictive Value": "NPV",
    "negative_predictive_value": "NPV",
    "False Negative Rate": "FNR",
    "false_negative_rate": "FNR",
    "True Negative Rate": "TNR",
    "true_negative_rate": "TNR",
    "False Discovery Rate": "FDR",
    "false_discovery_rate": "FDR",
    "Total Positives Test": "TP TEST",
    "total_positives_test": "TP TEST",
    "Total Negatives Test": "TN TEST",
    "total_negatives_test": "TN TEST",
    "Total Positives Reference": "TP REF",
    "total_positives_reference": "TP REF",
    "total Negatives Reference": "TN REF",
    "total_negatives_reference": "TN REF"
}


def parse_args():
    # Read in base directory
    parser = argparse.ArgumentParser()
    parser.add_argument("directory",
                        help="Give the path to the directory containing json score files",
                        type=str)
    parser.add_argument("-d", "--debug", action="store_true",
                        help="Turn debug mode on, eg. for live reloading.")
    parser.add_argument("-x", "--expose", action="store_true",
                        help="Make server externally visible")
    parser.add_argument("-p", "--port", default=5000, type=int,
                        help="Port to start the server on (5000 by default)")
    args = parser.parse_args()
    directory = args.directory
    if directory[-1] == os.sep:
        directory = directory[:-1]

    return args, directory


def create_flask_app(directory):
    # The actual flask app lives in the package directory. The blueprint allows us
    # to specify an additional static folder and we use that to give access to the
    # experiment files
    app = Flask(__name__, static_folder=os.path.join(os.path.dirname(__file__), "static"))
    blueprint = Blueprint("data", __name__, static_url_path=directory, static_folder=directory)
    app.register_blueprint(blueprint)

    return app


def register_url_routes(app, directory):
    app.add_url_rule("/", "overview", lambda: overview(directory), methods=["GET"])


def get_valid_json(directory):

    valid_json = []

    for f_name in os.listdir(directory):
        f_path = os.path.join(directory, f_name)
        try:
            with open(f_path, "r") as f_file:
                f_data = json.load(f_file)
                for r in REQUIRED_ENTRIES:
                    assert r in f_data
                valid_json.append(f_data)
        except Exception as e:
            continue

    return valid_json


def handle_json(valid_json):

    # get all labels and metrics
    all_labels = []
    all_metrics = []
    for item in valid_json:
        for label in item["results"]["mean"].keys():
            if label not in all_labels:
                all_labels.append(label)
            for metric in item["results"]["mean"][label].keys():
                if metric not in all_metrics:
                    all_metrics.append(metric)
    all_metrics.sort()

    converted = []
    for item in valid_json:
        row = []
        row.append(item["task"])
        row.append(item["name"])
        row.append(item.get("description", "--"))
        row.append(item.get("timestamp", "--"))
        row.append(item.get("author", "--"))
        row.append([])
        for label in all_labels:
            for metric in all_metrics:
                label_results = item["results"]["mean"].get(label)
                if label_results is not None:
                    row[-1].append(item["results"]["mean"][label].get(metric, "--"))
                else:
                    row[-1].append(np.nan)
        converted.append(row)

    return converted, all_labels, all_metrics


def overview(directory):

    content = {}

    # random quote :)
    try:
        r = requests.get("https://api.forismatic.com/api/1.0/?method=getQuote&format=json&lang=en")
        r = json.loads(r.content)
    except Exception as e:
        r = {"quoteText": "You must reload to see a quote.", "quoteAuthor": "Joel Spolsky"}
    if r["quoteAuthor"] == "":
        r["quoteAuthor"] = "Unknown"
    content["random_quote"] = r["quoteText"]
    content["random_quote_author"] = r["quoteAuthor"]

    valid_json = get_valid_json(directory)
    content["experiments"], content["labels"], content["metrics"] = handle_json(valid_json)
    content["abbreviations"] = ABBREVIATIONS

    return render_template("overview.html", **content)


def run():

    args, directory = parse_args()
    directory = os.path.abspath(directory)
    app = create_flask_app(directory)
    register_url_routes(app, directory)
    host = "0.0.0.0" if args.expose else "localhost"
    app.run(debug=args.debug, host=host, port=args.port)


if __name__ == "__main__":
    run()
