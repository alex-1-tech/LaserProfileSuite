#ifndef ANALYSYSPARAMS_H
#define ANALYSYSPARAMS_H

#include <variant>
#include "linearfitparams.h"
#include "houghparams.h"
#include "ransacparams.h"

/**
 * @struct AnalysisParams
 * @brief Union container for different analysis parameter types
 *
 * Provides type-safe storage for parameters of different analysis methods.
 * Using std::variant ensures only one parameter set is active at a time.
 */
struct AnalysisParams {
    std::variant<LinearFitParams, HoughParams, RANSACParams> params;

    /**
     * @brief Get parameters as specific type
     * @tparam T Parameter type (LinearFitParams, HoughParams, or RANSACParams)
     * @return Parameters of requested type
     * @throws std::bad_variant_access if wrong type requested
     */
    template<typename T>
    T getAs() const {
        return std::get<T>(params);
    }

    /**
     * @brief Get current parameter type index
     * @return 0=LinearFitParams, 1=HoughParams, 2=RANSACParams
     */
    size_t typeIndex() const {
        return params.index();
    }
};

#endif // ANALYSYSPARAMS_H
