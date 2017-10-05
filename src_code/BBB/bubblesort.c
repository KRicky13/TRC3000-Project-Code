void bubblesort(int arraysize, double * unsorted_array, int * flag_array)
{
	for (int x = 0; x < arraysize; x++)
	{
		for (int y = 0; y < arraysize - 1; y++)
		{
			if (unsorted_array[y] > unsorted_array[y + 1])
			{
				int temp = unsorted_array[y + 1];
				unsorted_array[y + 1] = unsorted_array[y];
				unsorted_array[y] = temp;

				int temp2 = flag_array[y + 1];
				flag_array[y + 1] = flag_array[y];
				flag_array[y] = temp2;
			}
		}
	}
}
