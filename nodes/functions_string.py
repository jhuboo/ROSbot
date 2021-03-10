""" Simple functions to manipulate Strings """

def is_rhyme(word1, word2, k):
    if (k == 0):
        return False

    if (len(word1) < k or len(word2) < k):
        return False

    rev_word1 = word1[::-1]
    rev_word2 = word2[::-1]
    return rev_word1[:k] == rev_word2[:k]


