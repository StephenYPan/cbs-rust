def partition(array, low, high):
    # function to find the partition position
    # choose the rightmost element as pivot
    pivot = array[high]
    # pointer for greater element
    # traverse through all elements
    # compare each element with pivot
    l = low
    for h in range(low, high):
        if array[h] <= pivot:
            array[l], array[h] = array[h], array[l]
            l = l + 1
    # swap the pivot element with the greater element specified by i
    array[l], array[high] = array[high], array[l]
    return l


def quickSort(array, low, high):
    # function to perform quicksort
    if low < high:
        # find pivot element such that
        # element smaller than pivot are on the left
        # element greater than pivot are on the right
        pi = partition(array, low, high)

        # recursive call on the left of pivot
        quickSort(array, low, pi - 1)
        # recursive call on the right of pivot
        quickSort(array, pi + 1, high)


data = [8, 7, 2, 1, 0, 9, 6]
print("Unsorted Array")
print(data)

size = len(data)

quickSort(data, 0, size - 1)

print('Sorted Array in Ascending Order:')
print(data)
