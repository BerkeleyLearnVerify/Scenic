# This files defiens the error table as a panda object
import pandas as pd
import numpy as np
from sklearn.decomposition import PCA
from sklearn.cluster import KMeans
from dotmap import DotMap
from collections import defaultdict
from kmodes.kmodes import KModes

class error_table():
    def __init__(self, space=None, table=None, column_type = None):
        assert space is not None or table is not None
        if space is not None:
            self.space= space
            self.column_names = []
            self.column_type = {}
            for i in range(space.fixedFlattenedDimension):
                self.column_names.append(space.meaningOfFlatCoordinate(i))
                self.column_type[space.meaningOfFlatCoordinate(i)] = \
                    space.coordinateIsNumerical(i)
            self.column_names.append("rho")
            self.column_type["rho"] = True # Set to numerical by default. Can be updated later.
            self.table = pd.DataFrame(columns=self.column_names)
            self.ignore_locs = []
        else:
            self.table = table
            self.column_names = table.columns
            if column_type is None:
                self.column_type = {col:True for col in self.column_names}
            else:
                self.column_type = column_type
            self.ignore_locs = []



    def update_column_names(self, column_names):
        assert len(self.table.columns) == len(column_names)
        self.table.columns = column_names
        self.column_names = column_names

    def update_error_table(self, sample, rho):
        sample = self.space.flatten(sample, fixedDimension=True)
        sample_dict = {}
        for k, v in zip(self.table.columns, list(sample)):
            if np.any(np.array(sample) == None):
                locs = np.where(np.array(sample) == None)
                self.ignore_locs = self.ignore_locs + list(locs[0])
            sample_dict[k] = float(v) if self.column_type[k] and v is not None else v
        if isinstance(rho, (list, tuple)):
            for i,r in enumerate(rho[:-1]):
                if "rho_" + str(i) not in self.column_names:
                    self.column_names.append("rho_"+str(i))
                    if isinstance(r, bool):
                        self.column_type["rho_" + str(i)] = False
                    else:
                        self.column_type["rho_" + str(i)] = True
                sample_dict["rho_"+str(i)]  = r
            sample_dict["rho"] = rho[-1]
            if isinstance(rho[-1], bool) and self.column_type["rho"]:
                self.column_type["rho"] = False
        else:
            sample_dict["rho"] = rho
            if isinstance(rho, bool) and self.column_type["rho"]:
                self.column_type["rho"] = False
        self.ignore_locs = list(set(tuple(self.ignore_locs)))
        new_row = pd.DataFrame(sample_dict, index=[0])
        self.table = pd.concat([self.table, new_row], ignore_index=True)


    def get_column_by_index(self, index):
        if isinstance(index, int):
            index = list([index])
        if len(index) < 1:
            print("No indices provided: returning all samples")
        elif max(index) >= len(self.table.columns):
            for i in index:
                if i >= len(self.table.columns):
                    index.remove(i)
            print("Tried to access index not in error table")
        if len(self.table) > 0:
            names_index = self.table.columns[index]
            return self.table[names_index]
        else:
            print("No entries in error table yet")
        return None

    def get_column_by_name(self, column_names):
        index = []
        if isinstance(column_names, str):
            if column_names in self.table.columns:
                index.append(column_names)
        else:
            for s in column_names:
                if s in self.table.columns:
                    index.append(s)
        return self.table[index]

    def get_samples_by_index(self, index):
        if isinstance(index, int):
            index = list([index])
        if max(index) >= len(self.table):
            print("Trying to access samples not in the table")
            for i in index:
                if i >= len(self.table):
                    index.remove(i)
        return self.table.iloc[index]

    def split_table(self, column_names=None):
        if column_names is None:
            column_names = self.column_names
        numerical, categorical = [], []
        for c in column_names:
            if self.column_type[c]:
                numerical.append(c)
            else:
                categorical.append(c)
        return self.get_column_by_name(numerical), self.get_column_by_name(categorical)

    def get_random_samples(self, count=5):
        if count > len(self.table):
            return list(range(len(self.table)))
        else:
            sample_ids = set()
            while len(sample_ids) < count:
                i = np.random.randint(len(self.table))
                sample_ids.add(i)
            return list(sample_ids)

    def build_normalized(self, column_names=None):
        if len(self.table) < 1:
            return pd.DataFrame(), pd.DataFrame()
        if column_names is None:
            column_names = self.column_names

        numerical, categorical = self.split_table(column_names=column_names)


        if len(categorical.columns) + len(numerical.columns) == 0:
            return pd.DataFrame(), pd.DataFrame()

        # Normalize tables (only for numerical table)
        stats = numerical.describe()

        normalized_dict = {r: (numerical[r] - stats[r]['min']) / (stats[r]['max'] - stats[r]['min'])
                         for r in numerical.columns}
        normalized_table = pd.DataFrame(normalized_dict)

        return normalized_table, categorical, \
               np.array([stats[r]['min'] for r in numerical.columns]),\
               np.array([stats[r]['max'] for r in numerical.columns])


    def build_standardized(self, column_names=None):
        if len(self.table) < 1:
            return pd.DataFrame(), pd.DataFrame()
        if column_names is None:
            column_names = self.column_names

        numerical, categorical = self.split_table(column_names=column_names)

        if len(categorical.columns) + len(numerical.columns) == 0:
            return pd.DataFrame(), pd.DataFrame()

        # Normalize tables (only for numerical table)
        stats = numerical.describe()

        standardized_dict = {r: (numerical[r] - stats[r]['mean']) / stats[r]['std']
                             for r in numerical.columns}
        standardized_table = pd.DataFrame(standardized_dict)

        return standardized_table, categorical, \
               np.array([stats[r]['mean'] for r in numerical.columns]),\
               np.array([stats[r]['std'] for r in numerical.columns])

    def dist_element(self, numerical, point_n):

        d=np.zeros(len(self.table))
        if len(numerical.columns) > 0:
            d = np.linalg.norm(numerical.values- point_n, axis=1)
        return d


    def k_clusters(self, column_names=None, k=None):
        if k is None or k >= len(self.table):
            return np.array(range(len(self.table)))

        if len(self.table) <=0 :
            return np.array(range(len(self.table)))

        numerical, categorical, min_elems, max_elems = self.build_normalized(column_names=column_names)
        range_elems = max_elems- min_elems
        result = defaultdict(dict)


        if len(categorical.columns) > 0:
            X = np.array(categorical.values)
            Y = np.array(numerical.values)
            kmodes = KModes(n_clusters=k, init='Huang', n_init=5, verbose=1)
            kmodes.fit_predict(X)
            centers_cat = kmodes.cluster_centroids_
            labels_cat = kmodes.labels_
            result['categorical'] = {'clusters':centers_cat, 'labels':labels_cat}
            for j in range(k):
                pos = Y[np.where(labels_cat == j)]
                kmeans = KMeans(n_clusters=min(len(pos), k), random_state=0).fit(pos)
                centers = kmeans.cluster_centers_
                labels = kmeans.labels_
                for label, center in enumerate(centers):
                    center = (center * range_elems) + min_elems
                    result[j][label] = center
                result[j]['labels'] = labels

        elif len(numerical.columns) > 0:
            X = np.array(numerical.values)
            kmeans = KMeans(n_clusters=k, random_state=0).fit(X)
            centers = kmeans.cluster_centers_
            labels = kmeans.labels_
            for label, center in enumerate(centers):
                center = (center*range_elems) + min_elems
                result['clusters'][label] = center
            result['labels'] = labels
        return result


    def k_closest_samples(self, column_names=None, k = None, dist_type=True):
        # dist_type is True for using normalized, False for standardized
        if k is None or k >= len(self.table):
             return np.array(range(len(self.table)))

        if dist_type:
            numerical, categorical, _, _ = self.build_normalized(column_names=column_names)
        else:
            numerical, categorical, _, _ = self.build_standardized(column_names=column_names)

        # Norm distance between table rows
        d_rows = np.zeros((len(self.table), len(self.table)))
        for i in range(len(self.table)):
            d_rows[i] = self.dist_element(numerical, numerical.values[i])

        # Now the row associated with the min sum is the largest set of correlated elements
        sum_rows = []
        correlated_rows = []
        for r in d_rows:
            ks = np.argpartition(r, k)[:k]
            sum_rows.append(sum(r[ks]))
            correlated_rows.append(ks)

        return correlated_rows[np.array(sum_rows).argmin()]

    def pca_analysis(self, column_names=None, n_components= 1):
        # Returns the direction of the principal component among the samples
        if len(self.table) < 1:
            return

        if column_names is None:
            column_names = self.column_names

        numerical, _ = self.split_table(column_names=column_names)


        # Do PCA analysis only on the numerical columns
        if len(numerical.columns) == 0:
            return
        pca_columns = []
        for c in numerical.columns:
            if c in self.table.columns and c not in self.table.columns[self.ignore_locs]:
                pca_columns.append(c)

        table = self.get_column_by_name(pca_columns)

        # PCA components
        if n_components > min(len(table), len(table.columns)):
            n_components = min(len(table), len(table.columns))

        pca = PCA(n_components=n_components)
        pca.fit(table)

        return {'columns':pca_columns, 'pivot': table.mean().values, 'directions': pca.components_}

    def analyze(self, analysis_params=None):
        analysis_data = DotMap()
        if analysis_params is None or ('pca' in analysis_params and analysis_params.pca) or 'pca' not in analysis_params:
            if analysis_params is not None and 'pca_params' in analysis_params:
                columns = analysis_params.pca_params.columns \
                    if 'columns' in analysis_params.pca_params else None
                n_components = analysis_params.pca_params.n_components \
                    if 'n_components' in analysis_params.pca_params else 1
            else:
                columns, n_components = None, 1
            analysis_data.pca = self.pca_analysis(column_names=columns, n_components=n_components)

        if analysis_params is None or ('k_closest' in analysis_params and analysis_params.k_closest) or 'k_closest' not in analysis_params:
            if analysis_params is not None and 'k_closest_params' in analysis_params:
                columns = analysis_params.k_closest_params.columns \
                    if 'columns' in analysis_params.k_closest_params else None
                k = analysis_params.k_closest_params.k \
                    if 'k' in analysis_params.k_closest_params else None
            else:
                columns, k = None, None
            analysis_data.k_closest = self.k_closest_samples(column_names=columns, k=k)


        if analysis_params is None or ('random' in analysis_params and analysis_params.random) or 'random' not in analysis_params:
            if analysis_params is not None and 'random_params' in analysis_params:
                count = analysis_params.random_params.count \
                    if 'count' in analysis_params.random_params else 5
            else:
                count = 5
            analysis_data.random = self.get_random_samples(count=count)

        if analysis_params is None or ('k_clusters' in analysis_params and analysis_params.k_clusters) or 'k_clusters' not in analysis_params:
            if analysis_params is not None and 'k_clusters_params' in analysis_params:
                columns = analysis_params.k_clusters_params.columns \
                    if 'columns' in analysis_params.k_clusters_params else None
                k = analysis_params.k_clusters_params.k \
                    if 'k' in analysis_params.k_clusters_params else None
            else:
                columns, k = None, None
            analysis_data.k_clusters = self.k_clusters(column_names=columns, k=k)
        return analysis_data





