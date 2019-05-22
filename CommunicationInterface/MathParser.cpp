#include "MathParser.hpp"

#pragma GCC diagnostic ignored "-Weffc++"
#pragma GCC diagnostic push
#define exprtk_disable_extended_optimisations
#include "exprtk.hpp"
#include <string>
namespace MathParser{
double parseDouble(std::string str){
	static exprtk::expression<double> expression;
	static exprtk::symbol_table<double> symbol_table;
	static exprtk::parser<double> parser;
	symbol_table.add_constants();
	expression.register_symbol_table(symbol_table);
	parser.compile(str,expression);
	return expression.value();
};
};
#pragma GCC diagnostic pop
