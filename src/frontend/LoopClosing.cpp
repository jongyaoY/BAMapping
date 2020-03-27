//
// Created by jojo on 26.03.20.
//

#include "LoopClosing.h"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
//#include "opencv2/core/eigen.hpp"

using namespace BAMapping::FrontEnd;

void LoopClosing::init(std::string voc_path, bool loadFromeText,Parameters params)
{
    std::cout<<"loading vocabulary..."<<std::endl;
    m_param = params;
    OrbVocabulary voc;
    if(loadFromeText)
        voc.loadFromTextFile(voc_path);
    else
    {
        OrbVocabulary voc_(9,3);
        voc_.load(voc_path);
        voc = voc_;
    }
    std::cout<<"done"<<std::endl;
    std::cout<<"initializing database..."<<std::endl;

    feature_db_ = new OrbDatabase(voc,false,0);

    std::cout<<"done"<<std::endl;

}

void LoopClosing::addToDataBase(std::vector<cv::Mat> frame_descriptors)
{
}

std::vector<size_t> LoopClosing::dectectAndGetCandidate(std::vector<cv::Mat> frame_descriptors)
{
    std::vector<size_t> matches;
    DBoW2::QueryResults ret;
    auto query_id = feature_db_->size();
    auto max_id = query_id - m_param.min_distance;
    if(max_id < 0)
    {
        feature_db_->add(frame_descriptors);
        return matches;
    }
    feature_db_->query(frame_descriptors,ret,m_param.max_db_results,max_id);
    feature_db_->add(frame_descriptors);

    if(!ret.empty())
    {
        auto ns_factor = ret[0].Score;
        removeLowScores(ret, m_param.alpha * ns_factor);
        for(auto& r : ret)
        {
            r.Score /= ns_factor;
        }

        std::vector<Island> islands;
        computeIslands(ret,islands);
        if(!islands.empty())
        {
            const Island& island = *std::max_element(islands.begin(), islands.end());
            updateTemporalWindow(island, query_id);
            if(getConsistentEntries() > m_param.k) // candidate loop detected
            {
//                std::cout<<island.toString()<<std::endl;
                matches.push_back(island.best_entry);

            }
        }
    }
//    for(auto r : ret)
//    {
//        matches.push_back(r.Id);
//
//    }
    return matches;
}

void LoopClosing::computeIslands(DBoW2::QueryResults &q, vector<Island> &islands)
{
    islands.clear();

    if(q.size() == 1)
    {
        islands.push_back(Island(q[0].Id, q[0].Id, calculateIslandScore(q, 0, 0)));
        islands.back().best_entry = q[0].Id;
        islands.back().best_score = q[0].Score;
    }
    else if(!q.empty())
    {
        // sort query results in ascending order of ids
        std::sort(q.begin(), q.end(), Result::ltId);

        // create long enough islands
        QueryResults::const_iterator dit = q.begin();
        int first_island_entry = dit->Id;
        int last_island_entry = dit->Id;

        // these are indices of q
        unsigned int i_first = 0;
        unsigned int i_last = 0;

        double best_score = dit->Score;
        EntryId best_entry = dit->Id;

        ++dit;
        for(unsigned int idx = 1; dit != q.end(); ++dit, ++idx)
        {
            if((int)dit->Id - last_island_entry < m_param.max_intragroup_gap)
            {
                // go on until find the end of the island
                last_island_entry = dit->Id;
                i_last = idx;
                if(dit->Score > best_score)
                {
                    best_score = dit->Score;
                    best_entry = dit->Id;
                }
            }
            else
            {
                // end of island reached
                int length = last_island_entry - first_island_entry + 1;
                if(length >= m_param.min_matches_per_group)
                {
                    islands.push_back( Island(first_island_entry, last_island_entry,
                                               calculateIslandScore(q, i_first, i_last)) );

                    islands.back().best_score = best_score;
                    islands.back().best_entry = best_entry;
                }

                // prepare next island
                first_island_entry = last_island_entry = dit->Id;
                i_first = i_last = idx;
                best_score = dit->Score;
                best_entry = dit->Id;
            }
        }
        // add last island
        if(last_island_entry - first_island_entry + 1 >=
           m_param.min_matches_per_group)
        {
            islands.push_back( Island(first_island_entry, last_island_entry,
                                       calculateIslandScore(q, i_first, i_last)) );

            islands.back().best_score = best_score;
            islands.back().best_entry = best_entry;
        }
    }
}

double LoopClosing::calculateIslandScore(const DBoW2::QueryResults &q, unsigned int i_first,
                                         unsigned int i_last) const
 {
     // get the sum of the scores
     double sum = 0;
     while(i_first <= i_last) sum += q[i_first++].Score;
     return sum;
 }
void LoopClosing::updateTemporalWindow(const BAMapping::FrontEnd::LoopClosing::Island &matched_island,
                                       DBoW2::EntryId entry_id)
{
    // if m_window.nentries > 0, island > m_window.last_matched_island and
    // entry_id > m_window.last_query_id hold

    if(m_window.nentries == 0 || int(entry_id - m_window.last_query_id)
                                 > m_param.max_distance_between_queries)
    {
        m_window.nentries = 1;
    }
    else
    {
        EntryId a1 = m_window.last_matched_island.first;
        EntryId a2 = m_window.last_matched_island.last;
        EntryId b1 = matched_island.first;
        EntryId b2 = matched_island.last;

        bool fit = (b1 <= a1 && a1 <= b2) || (a1 <= b1 && b1 <= a2);

        if(!fit)
        {
            int d1 = (int)a1 - (int)b2;
            int d2 = (int)b1 - (int)a2;
            int gap = (d1 > d2 ? d1 : d2);

            fit = (gap <= m_param.max_distance_between_groups);
        }

        if(fit) m_window.nentries++;
        else m_window.nentries = 1;
    }

    m_window.last_matched_island = matched_island;
    m_window.last_query_id = entry_id;
}