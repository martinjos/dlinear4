#include <istream>
#include <string>
#include <vector>

#include "dreal/smt2/location.hh"
#include "dreal/smt2/scanner.h"
#include "dreal/solver/context.h"
#include "dreal/util/scoped_unordered_map.h"
#include "dreal/util/optional.h"

namespace dreal {

/** The Smt2Driver class brings together all components. It creates an
 * instance of the Parser and Scanner classes and connects them. Then
 * the input stream is fed into the scanner object and the parser gets
 * it's token sequence. Furthermore the driver object is available in
 * the grammar rules as a parameter. Therefore the driver class
 * contains a reference to the structure into which the parsed data is
 * saved. */
class Smt2Driver {
 public:
  /// construct a new parser driver context
  Smt2Driver() = default;
  explicit Smt2Driver(Context context);

  /** Invoke the scanner and parser for a stream.
   * @param in	input stream
   * @param sname	stream name for error messages
   * @return		true if successfully parsed
   */
  bool parse_stream(std::istream& in,
                    const std::string& sname = "stream input");

  /** Invoke the scanner and parser on an input string.
   * @param input	input string
   * @param sname	stream name for error messages
   * @return		true if successfully parsed
   */
  bool parse_string(const std::string& input,
                    const std::string& sname = "string stream");

  /** Invoke the scanner and parser on a file. Use parse_stream with a
   * std::ifstream if detection of file reading errors is required.
   * @param filename	input file name
   * @return		true if successfully parsed
   */
  bool parse_file(const std::string& filename);

  // To demonstrate pure handling of parse errors, instead of
  // simply dumping them on the standard error output, we will pass
  // them to the driver using the following two member functions.

  /** Error handling with associated line number. This can be modified to
   * output the error e.g. to a dialog box. */
  static void error(const location& l, const std::string& m);

  /** General error handling. This can be modified to output the error
   * e.g. to a dialog box. */
  static void error(const std::string& m);

  /// Calls context_.CheckSat() and print proper output messages to cout.
  void CheckSat();

  /// Register a variable with name @p name and sort @p s in the scope. Note
  /// that it does not declare the variable in the context.
  Variable RegisterVariable(const std::string& name, Sort sort);

  /// Declare a variable with name @p name and sort @p sort.
  Variable DeclareVariable(const std::string& name, Sort sort);

  /// Declare a variable with name @p name and sort @p sort which is bounded by
  /// an interval `[lb, ub]`.
  void DeclareVariable(const std::string& name, Sort sort, const Term& lb,
                       const Term& ub);

  /// Declare a new variable with label @p name that is globally unique and
  /// cannot occur in an SMT-LIBv2 file.
  Variable DeclareLocalVariable(const std::string& name, Sort sort);

  // Define a constant within the current scope.
  void DefineLocalConstant(const std::string& name, const Expression& value);

  /// Returns a representation of a model computed by the solver in
  /// response to an invocation of the check-sat.
  void GetModel();

  class VariableOrConstant {
   public:
    explicit VariableOrConstant(const Variable& var) : var_{var}, is_var_{true} {}
    explicit VariableOrConstant(const Expression& expr) : expr_{expr}, is_var_{false} {}
    const Variable& variable() const { return var_; }
    const Expression& expression() const { return expr_; }
    bool is_variable() const { return is_var_; }
   private:
    Variable var_;
    Expression expr_;
    bool is_var_;
  };

  /// Returns a variable or constant expression associated with a name @p name.
  ///
  /// @throws if no variable or constant expression is associated with @p name.
  const VariableOrConstant& lookup_variable(const std::string& name);

  void PushScope() { scope_.push(); }

  void PopScope() { scope_.pop(); }

  static Variable ParseVariableSort(const std::string& name, Sort s);

  std::string MakeUniqueName(const std::string& name);

  bool trace_scanning() const { return trace_scanning_; }
  void set_trace_scanning(bool b) { trace_scanning_ = b; }

  bool trace_parsing() const { return trace_parsing_; }
  void set_trace_parsing(bool b) { trace_parsing_ = b; }

  Context& mutable_context() { return context_; }

  std::string& mutable_streamname() { return streamname_; }

  /** Pointer to the current scanenr instance, this is used to connect the
   * parser to the scanner. It is used in the yylex macro. */
  Smt2Scanner* scanner_{nullptr};

 private:
  /// enable debug output in the flex scanner
  bool trace_scanning_{false};

  /// enable debug output in the bison parser
  bool trace_parsing_{false};

  /** Scoped map from a string to a corresponding Variable or constant Expression. */
  ScopedUnorderedMap<std::string, VariableOrConstant> scope_;

  /// Sequential value concatenated to names to make them unique.
  int64_t nextUniqueId_{};

  /// stream name (file or input stream) used for error messages.
  std::string streamname_;

  /** The context filled during parsing of the expressions. */
  Context context_;
};

}  // namespace dreal
