(set-logic QF_NRA)
(declare-const a Real)
(assert (= a 2))
(assert
  (let ((b (* a 4))
        (c 6))
    (>= (* b c) 10)))
(check-sat)
(exit)
