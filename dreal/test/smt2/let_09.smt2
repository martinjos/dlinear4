(set-logic QF_NRA)
(declare-const a Real)
(assert (= a 2))
(assert
  (let ((b 10))
    (let ((b a))
      (= b 10))))
(check-sat)
(exit)
