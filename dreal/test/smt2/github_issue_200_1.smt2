;; From https://github.com/dreal/dreal4/issues/200
(set-logic QF_NRA)
(set-option :worklist-fixpoint true)
(declare-const r31415926 Real)
(declare-const r31415927 Real)
(assert (<= r31415926 r31415927))
(declare-const r1 Real)
(check-sat)
(declare-const r5 Real)
(assert (not (= r1 r5)))
(check-sat)
