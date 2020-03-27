//
// Created by jojo on 26.03.20.
//

#ifndef BAMAPPING_LOOPCLOSING_H
#define BAMAPPING_LOOPCLOSING_H

//#include "../Frame.h"
#include "opencv2/opencv.hpp"
#include "DBoW2.h"


namespace BAMapping
{
    namespace FrontEnd
    {   using namespace DBoW2;
        class LoopClosing
        {
        public:
            struct Parameters
            {
                Parameters()
                {
                    alpha = 0.3;
                    k = 3;
                    max_db_results = 50;
                    min_nss_factor = 0.005;
                    min_distance = 20;
                    min_matches_per_group = 1;
                    max_intragroup_gap = 3;
                    max_distance_between_queries = 2;
                    max_distance_between_groups = 3;
                }
                int max_distance_between_groups;
                int max_distance_between_queries;
                /// Min number of close matches to consider some of them
                int min_matches_per_group;
                /// Max separation between matches to consider them of the same group
                int max_intragroup_gap;
                /// Distance between entries to be consider a match
                int min_distance;
                /// Alpha threshold
                float alpha;
                /// Min consistent matches to pass the temporal check
                int k;
                int max_db_results;
                /// Min raw score between current entry and previous one to consider a match
                float min_nss_factor;
            };

            void init(const std::string voc_path,bool loadFromeText = true,Parameters params = Parameters());
            void addToDataBase(std::vector<cv::Mat> frame_descriptors);
            std::vector<size_t> dectectAndGetCandidate(std::vector<cv::Mat> frame_descriptors);
        public:
            struct Island
            {
                /// Island starting entry
                EntryId first;
                /// Island ending entry
                EntryId last;
                /// Island score
                double score; // score of island
                /// Entry of the island with the highest score
                EntryId best_entry; // id and score of the entry with the highest score
                /// Highest single score in the island
                double best_score;  // in the island
                Island(){}
                Island(EntryId f, EntryId l): first(f), last(l){}
                Island(EntryId f, EntryId l, double s): first(f), last(l), score(s){}
                /**
                 * Says whether this score is less than the score of another island
                 * @param b
                 * @return true iff this score < b.score
                 */
                inline bool operator < (const Island &b) const
                {
                    return this->score < b.score;
                }

                /**
                 * Says whether this score is greater than the score of another island
                 * @param b
                 * @return true iff this score > b.score
                 */
                inline bool operator > (const Island &b) const
                {
                    return this->score > b.score;
                }

                /**
                 * Returns true iff a > b
                 * This function is used to sort in descending order
                 * @param a
                 * @param b
                 * @return a > b
                 */
                static inline bool gt(const Island &a, const Island &b)
                {
                    return a.score > b.score;
                }

                /**
                 * Returns true iff entry ids of a are less then those of b.
                 * Assumes there is no overlap between the islands
                 * @param a
                 * @param b
                 * @return a.first < b.first
                 */
                static inline bool ltId(const Island &a, const Island &b)
                {
                    return a.first < b.first;
                }

                /**
                 * Returns the length of the island
                 * @return length of island
                 */
                inline int length() const { return last - first + 1; }

                /**
                 * Returns a printable version of the island
                 * @return printable island
                 */
                std::string toString() const
                {
                    stringstream ss;
                    ss << "[" << first << "-" << last << ": " << score << " | best: <"
                       << best_entry << ": " << best_score << "> ] ";
                    return ss.str();
                }
            };
            /// Temporal consistency window
            struct TemporalWindow
            {
                /// Island matched in the last query
                Island last_matched_island;
                /// Last query id
                EntryId last_query_id;
                /// Number of consistent entries in the window
                int nentries;

                /**
                 * Creates an empty temporal window
                 */
                TemporalWindow(): nentries(0) {}
            };
        private:
            OrbDatabase* feature_db_;
            Parameters m_param;
            TemporalWindow m_window;
        private:
            void computeIslands(QueryResults &q, vector<Island> &islands);
            double calculateIslandScore(const QueryResults &q, unsigned int i_first,
                                        unsigned int i_last) const;
            void updateTemporalWindow(const Island &matched_island, EntryId entry_id);
            inline int getConsistentEntries() const
            {
                return m_window.nentries;
            }
            void removeLowScores(QueryResults &q,
            double threshold) const
            {
                // remember scores in q are in descending order now
                //QueryResults::iterator qit =
                //  lower_bound(q.begin(), q.end(), threshold, Result::geqv);

                Result aux(0, threshold);
                QueryResults::iterator qit =
                        lower_bound(q.begin(), q.end(), aux, Result::geq);

                // qit = first element < m_alpha_minus || end

                if(qit != q.end())
                {
                    int valid_entries = qit - q.begin();
                    q.resize(valid_entries);
                }
            }
        };
    }
}



#endif //BAMAPPING_LOOPCLOSING_H
