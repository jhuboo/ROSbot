""" Simple functions to manipulate strings """

# Replace the functions below with your implementation as described in the assignment


def is_rhyme(word1, word2, k):
    """
    Returns True if the last k letters of the two words are the same (case sensitive).
    Automatically returns False if either word contains less than k letters.
    """
    if (k == 0):        # Cannot compare if k is 0
        return False

    if (len(word1) < k or len(word2) < k):      # Return False if either word
        return False                            # contains less than k letters

    rev_word1 = word1[::-1]     # Reverse word1
    rev_word2 = word2[::-1]     # Reverse word2

    # Compare first k chars of reversed word1 and reversed word2
    # Equivalent of comparing last k chars of word 1 and word2
    return rev_word1[:k] == rev_word2[:k]


# Test Cases
# print(is_rhyme("hello", "world", 7))    # False
# print(is_rhyme("hello", "llo", 3))      # True
# print(is_rhyme("ello", "ello", 0))      # False
# print(is_rhyme("elo", "ello", 3))       # False
# print(is_rhyme("ello", "ello", 4))      # True
# print(is_rhyme("ello", "ello", 5))      # False
