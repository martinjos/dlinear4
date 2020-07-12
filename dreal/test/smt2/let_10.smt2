(set-logic QF_NRA)
(declare-const a Real)
(assert (= a 2))
(assert
  (let ((b a))
    (let ((b 10)
          (c b))  ; c should be set to 2. If it's 10, this is a bug.
      (= c 2))))
(check-sat)
(exit)
